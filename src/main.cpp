#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <Preferences.h>
#include "config.h"
#include "corolla_bitmap.h"
#include "obd.h"

template<uint8_t CS_PIN>
class GC9A01 : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel;
  lgfx::Bus_SPI      _bus;
public:
  GC9A01() {
    { auto cfg = _bus.config();
      cfg.spi_host   = VSPI_HOST; cfg.freq_write = 27000000;
      cfg.pin_sclk   = TFT_SCK;  cfg.pin_mosi   = TFT_MOSI;
      cfg.pin_miso   = -1;       cfg.pin_dc      = TFT_DC;
      _bus.config(cfg); _panel.setBus(&_bus);
    }
    { auto cfg = _panel.config();
      cfg.pin_cs = CS_PIN; cfg.pin_rst = -1; cfg.pin_busy = -1;
      cfg.memory_width = cfg.panel_width = 240;
      cfg.memory_height = cfg.panel_height = 240;
      cfg.rgb_order = false; _panel.config(cfg);
    }
    setPanel(&_panel);
  }
};

static GC9A01<TFT1_CS> disp1;
static GC9A01<TFT2_CS> disp2;
static GC9A01<TFT3_CS> disp3;
static LGFX_Sprite spr(&disp1);
static OBDReader obd;

static void drawDialLabels(int cx, int cy, int radius, float minVal, float maxVal, int steps,
                           const char* suffix, uint32_t col,
                           float startDeg=135.0f, float sweepDeg=270.0f){
  spr.setTextDatum(textdatum_t::middle_center); spr.setTextSize(1); spr.setTextColor(col);
  for(int i=0;i<=steps;i++){
    float t=i/(float)steps, val=minVal+(maxVal-minVal)*t;
    float deg=startDeg+sweepDeg*t, rad=deg*DEG_TO_RAD;
    int x=cx+(int)(cosf(rad)*radius), y=cy+(int)(sinf(rad)*radius);
    String s=String((int)val); if(suffix&&suffix[0]) s+=suffix;
    spr.drawString(s,x,y);
  }
}

static SemaphoreHandle_t dataMutex;
struct CarData {
  float rpm, speed_kph, coolant_c, load_pct, throttle, iat_c, batt_v, maf_g_s, stft, ltft, timing;
  int gear; bool obd_ok;
} carData;
static volatile uint8_t mode1=0, mode2=0, mode3=0;

static constexpr float WARN_TEMP_C = 105.0f;

static float avgMPG=0.0f;
static int   avgMPGCnt=0;
static float fuelUsed_gal=0.0f;
static float lastSavedFuel=0.0f;
static uint32_t lastFuelSaveMs=0;
static float odomMiles=0.0f;   // persisted to NVS
static float tripMiles=0.0f;   // resets each power cycle
static float odomAccum=0.0f;   // sub-0.1 mile accumulator (file scope so renderOdometer can read it)

static uint32_t lp1=0, lp2=0, lp3=0;
static bool     fuelResetArmed=false;   // true while BTN3 held on fuel screen
static uint32_t btn3HoldStart=0;
static void checkButtons(){
  static uint32_t lastPrint=0;
  if(millis()-lastPrint>500){
    Serial.printf("BTN1:%d BTN2:%d BTN3:%d modes: %d %d %d\n",
                  digitalRead(BTN1_PIN),digitalRead(BTN2_PIN),digitalRead(BTN3_PIN),
                  mode1,mode2,mode3);
    lastPrint=millis();
  }
  uint32_t n=millis();
  if(!digitalRead(BTN1_PIN)&&n-lp1>250){mode1=(mode1+1)%4;lp1=n;}
  if(!digitalRead(BTN2_PIN)&&n-lp2>250){mode2=(mode2+1)%4;lp2=n;}

  // BTN3: short press cycles screens; 3-second hold on fuel screen resets counter
  bool b3=!digitalRead(BTN3_PIN);
  if(b3){
    if(btn3HoldStart==0) btn3HoldStart=n;
    if(mode3==3&&!fuelResetArmed&&(n-btn3HoldStart>=3000)){
      fuelResetArmed=true;
      fuelUsed_gal=0.0f;
      lastSavedFuel=0.0f;
      lastFuelSaveMs=millis();
      Preferences p;p.begin("carcoach",false);p.putFloat("fuel_gal",0.0f);p.end();
      Serial.println("Fuel counter reset.");
    }
  } else {
    if(btn3HoldStart>0&&(n-btn3HoldStart)<250) {
      // short press — only cycle if we didn't just do a reset
      if(!fuelResetArmed) { mode3=(mode3+1)%4; lp3=n; }
    }
    btn3HoldStart=0; fuelResetArmed=false;
  }
}

#if DEMO_MODE
static void updateDemo(){
  static uint32_t t0=millis(); float t=(millis()-t0)/1000.0f;
  const float half=6.0f; float phase=fmodf(t,half*2.0f);
  float mph=(phase<half)?80.0f*(phase/half):80.0f*(1.0f-((phase-half)/half));
  float kph=mph*1.60934f,rpm=850.0f+mph*34.0f+180.0f*sinf(t*2.0f);
  float cool=88.0f+4.0f*sinf(t*0.2f),load=18.0f+mph*0.7f;
  float thr=(phase<half)?(20.0f+mph*0.75f):(10.0f+mph*0.45f);
  float iat=32.0f+2.0f*sinf(t*0.4f),batt=14.1f+0.15f*sinf(t*0.3f);
  if(rpm<800)rpm=800; if(load>100)load=100; if(thr>100)thr=100;
  int gear=0;
  if(mph>2){if(mph<12)gear=1;else if(mph<22)gear=2;else if(mph<35)gear=3;else if(mph<52)gear=4;else gear=5;}
  xSemaphoreTake(dataMutex,portMAX_DELAY);
  carData={rpm,kph,cool,load,thr,iat,batt,0.0f,gear,true};
  xSemaphoreGive(dataMutex);
}
#endif

static uint32_t C(uint8_t r,uint8_t g,uint8_t b){return spr.color565(r,g,b);}
static void glowText(const String& s,int x,int y,int sz,uint32_t bright,uint32_t glow){
  spr.setTextSize(sz);spr.setTextDatum(textdatum_t::middle_center);spr.setTextColor(glow);
  for(int dx=-1;dx<=1;dx++)for(int dy=-1;dy<=1;dy++)if(dx||dy)spr.drawString(s,x+dx,y+dy);
  spr.setTextColor(bright);spr.drawString(s,x,y);
}
static void segArc(float val,float mn,float mx,int ns,float z1p,float z2p,
                   uint32_t c1,uint32_t c2,uint32_t c3,uint32_t dim,int ro=110,int ri=90){
  float pct=constrain((val-mn)/(mx-mn),0.0f,1.0f); int lit=(int)(pct*ns);
  for(int i=0;i<ns;i++){
    float a0=135.0f+(float)i/ns*270.0f+1.2f,a1=135.0f+(float)(i+1)/ns*270.0f-1.2f;
    float sp=(float)i/ns; uint32_t cl=(sp<z1p)?c1:(sp<z2p)?c2:c3;
    spr.fillArc(120,120,ro,ri,a0,a1,i<lit?cl:dim);
  }
}
static void cornerDeco(){
  uint32_t c=C(200,40,0);
  spr.drawLine(5,5,20,5,c);spr.drawLine(5,5,5,20,c);
  spr.drawLine(235,5,220,5,c);spr.drawLine(235,5,235,20,c);
  spr.drawLine(5,235,20,235,c);spr.drawLine(5,235,5,220,c);
  spr.drawLine(235,235,220,235,c);spr.drawLine(235,235,235,220,c);
}
static void modeDots(int cur,int total=3){
  int sx=120-(total-1)*5;
  for(int i=0;i<total;i++) spr.fillCircle(sx+i*10,225,3,i==cur?C(230,40,0):C(60,20,20));
}

// Center scene
static float laneShift=0.0f; static uint32_t lastSceneTick=0;
static void updateCenterSceneMotion(float speed_kph){
  uint32_t now=millis();
  float dt=(lastSceneTick==0)?0.033f:(now-lastSceneTick)/1000.0f;
  lastSceneTick=now;
  laneShift-=speed_kph*0.621371f*5.0f*dt;
  while(laneShift<-30.0f)laneShift+=30.0f;
  while(laneShift>30.0f)laneShift-=30.0f;
}
struct Pt{int16_t x,y;};
template<size_t N>
static void drawPolyline(const Pt(&pts)[N],int ox,int oy,float s,uint32_t col){
  for(size_t i=0;i+1<N;++i){
    spr.drawLine(ox+(int)(pts[i].x*s),oy+(int)(pts[i].y*s),
                 ox+(int)(pts[i+1].x*s),oy+(int)(pts[i+1].y*s),col);
  }
}
static void drawPerspectiveGrid(int horizonY){
  spr.drawFastHLine(0,horizonY,240,C(255,0,0));
  for(int i=1;i<=6;i++){
    float t=(float)i/6.0f; int y=horizonY+(int)((239-horizonY)*t*t);
    uint8_t g=(uint8_t)max(0,(int)(40-t*20));
    spr.drawFastHLine(0,y,240,C(255,g,0));
  }
  float spacing=30.0f,offset=fmodf(laneShift,spacing);
  if(offset>0)offset-=spacing;
  for(float bx=offset;bx<240.0f+spacing;bx+=spacing){
    float tx=120.0f+(bx-120.0f)*0.25f;
    spr.drawLine((int)bx,239,(int)tx,horizonY,C(255,0,0));
  }
}
static void draw1BPPBitmap(int x,int y,const uint8_t* data,int w,int h,uint32_t color,bool flipX=false){
  int bpr=(w+7)/8;
  for(int yy=0;yy<h;yy++){
    for(int xb=0;xb<bpr;xb++){
      uint8_t b=pgm_read_byte(&data[yy*bpr+xb]); if(!b)continue;
      for(int bit=0;bit<8;bit++){
        int xx=xb*8+bit;
        if(xx<w&&(b&(1<<bit))){
          int dx=flipX?(x+(w-1-xx)):(x+xx);
          spr.drawPixel(dx,y+yy,color);
        }
      }
    }
  }
}

static void drawAlert(const String& line1,const String& line2,uint32_t bg,uint32_t fg){
  spr.fillCircle(120,120,85,bg);
  spr.setTextDatum(textdatum_t::middle_center);
  spr.setTextColor(fg); spr.setTextSize(2);
  spr.drawString(line1,120,100);
  spr.drawString(line2,120,138);
}

static void renderCorollaScene(const CarData& d){
  spr.fillScreen(TFT_BLACK);
  const int horizonY=160;
  drawPerspectiveGrid(horizonY);
  float mph=d.speed_kph*0.621371f;
  spr.setTextDatum(textdatum_t::middle_center); spr.setTextSize(4); spr.setTextColor(TFT_WHITE);
  spr.drawString(String((int)roundf(mph)),120,60);
  int carX=(240-COROLLA_W)/2, carY=horizonY-COROLLA_H+2;
  draw1BPPBitmap(carX,carY,corolla_bits,COROLLA_W,COROLLA_H,TFT_WHITE,true);
}

static void renderRPM(const CarData& d){
  spr.fillScreen(C(5,2,15));cornerDeco();
  segArc(d.rpm,0,7000,36,0.57f,0.85f,C(220,0,0),C(230,180,0),C(255,0,180),C(25,15,35));
  spr.fillArc(120,120,115,108,135.0f+0.85f*270.0f,135.0f+270.0f,C(0,255,0));
  spr.drawCircle(120,120,87,C(220,40,0));
  drawDialLabels(120,120,74,0,7,7,"k",TFT_WHITE);
  glowText(String((int)d.rpm),120,100,3,C(230,40,0),C(70,10,0));
  glowText("RPM",120,130,1,C(200,40,0),C(60,0,0));
  String gs=d.gear>0?"GEAR "+String(d.gear):"GEAR N";
  glowText(gs,120,155,2,C(255,140,0),C(80,30,0));
  spr.setTextDatum(textdatum_t::middle_center);spr.setTextSize(1);spr.setTextColor(C(100,60,60));
  spr.drawString("LOAD",120,178);
  spr.fillRoundRect(80,184,80,6,3,C(20,20,35));
  spr.fillRoundRect(80,184,(int)(d.load_pct/100.0f*80),6,3,C(220,40,0));
  modeDots(0,5);
}
static void renderLoad(const CarData& d){
  spr.fillScreen(C(5,2,15));cornerDeco();
  segArc(d.load_pct,0,100,30,0.6f,0.85f,C(220,0,0),C(230,180,0),C(255,0,180),C(25,15,35));
  spr.drawCircle(120,120,87,C(255,140,0));
  glowText(String((int)d.load_pct)+"%",120,100,3,C(255,140,0),C(80,30,0));
  glowText("ENGINE LOAD",120,135,1,C(200,100,0),C(50,20,0));
  modeDots(2,5);
}
static void renderMPG(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  float mph=d.speed_kph*0.621371f;
  float mpg=(mph>1.0f&&d.maf_g_s>0.1f)
            ?constrain(d.speed_kph*7.103f/d.maf_g_s,0.0f,99.9f):0.0f;
  segArc(mpg,0,50,30,0.2f,0.6f,C(220,0,0),C(230,180,0),C(0,200,80),C(25,15,35));
  spr.drawCircle(120,120,87,C(0,190,60));
  drawDialLabels(120,120,74,0,50,5,"",TFT_WHITE);
  if(mph<1.0f||d.maf_g_s<0.1f)
    glowText("--",120,100,3,C(80,80,80),C(20,20,20));
  else
    glowText(String((int)mpg),120,100,3,C(255,255,255),C(60,60,60));
  glowText("MPG",120,130,1,C(255,255,255),C(60,60,60));
  modeDots(1,5);
}
static void renderSystems(const CarData& d){
  spr.fillScreen(C(5,2,15));cornerDeco();
  uint32_t tc=d.coolant_c<95?C(220,0,0):d.coolant_c<105?C(230,180,0):C(255,0,180);
  segArc(d.coolant_c,40,120,28,0.69f,0.81f,C(220,0,0),C(230,180,0),C(255,0,180),C(25,15,35));
  spr.drawCircle(120,120,87,tc);
  drawDialLabels(120,120,74,40,120,4,"",TFT_WHITE);
  glowText(String((int)d.coolant_c),108,110,3,tc,C(30,30,0));
  spr.drawCircle(140,97,4,tc);glowText("C",154,110,3,tc,C(30,30,0));
  glowText("ENGINE TEMP",120,143,1,C(150,100,0),C(30,20,0));
  spr.setTextSize(1);spr.setTextDatum(textdatum_t::middle_center);spr.setTextColor(C(80,60,80));
  char iatbuf[16];snprintf(iatbuf,sizeof(iatbuf),"IAT %.0fC",d.iat_c);
  spr.drawString(iatbuf,120,195);
  modeDots(0,4);
}
static void renderFuelTrim(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  // Long term trim as main arc (-25% to +25%)
  float ltft_c=constrain(d.ltft,-25.0f,25.0f);
  segArc(ltft_c+25.0f,0,50,30,0.35f,0.55f,C(0,200,80),C(230,180,0),C(220,0,0),C(25,15,35));
  uint32_t lc=fabsf(d.ltft)<5?TFT_WHITE:fabsf(d.ltft)<12?C(230,180,0):C(220,0,0);
  uint32_t sc=fabsf(d.stft)<5?TFT_WHITE:fabsf(d.stft)<12?C(230,180,0):C(220,0,0);
  spr.drawCircle(120,120,87,lc);
  glowText("LONG TERM",120,75,1,C(140,100,0),C(35,25,0));
  char lb[12];snprintf(lb,sizeof(lb),"%+.1f%%",d.ltft);
  glowText(String(lb),120,97,2,lc,C(20,20,20));
  spr.drawFastHLine(60,118,120,C(40,30,40));
  glowText("SHORT TERM",120,140,1,C(140,100,0),C(35,25,0));
  char sb[12];snprintf(sb,sizeof(sb),"%+.1f%%",d.stft);
  glowText(String(sb),120,162,2,sc,C(20,20,20));
  glowText("FUEL TRIM",120,200,1,C(180,100,0),C(50,25,0));
  modeDots(2,4);
}
static void renderThrottle(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  float eff=constrain(d.throttle,0.0f,100.0f);
  segArc(eff,0,100,30,0.5f,0.8f,C(0,180,220),C(230,180,0),C(220,0,0),C(25,15,35));
  spr.drawCircle(120,120,87,C(0,160,200));
  drawDialLabels(120,120,74,0,100,5,"",TFT_WHITE);
  glowText(String((int)eff)+"%",120,100,3,C(255,255,255),C(60,60,60));
  glowText("THROTTLE",120,135,1,C(255,255,255),C(60,60,60));
  modeDots(1,4);
}

static void renderAvgMPG(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  float display=constrain(avgMPG,0.0f,50.0f);
  segArc(display,0,50,30,0.2f,0.6f,C(220,0,0),C(230,180,0),C(0,200,80),C(25,15,35));
  spr.drawCircle(120,120,87,C(0,180,60));
  drawDialLabels(120,120,74,0,50,5,"",TFT_WHITE);
  if(avgMPGCnt==0)
    glowText("--",120,95,3,C(80,80,80),C(20,20,20));
  else
    glowText(String((int)avgMPG),120,95,3,C(255,255,255),C(60,60,60));
  glowText("AVG MPG",120,130,1,C(255,255,255),C(60,60,60));
  spr.setTextSize(1); spr.setTextDatum(textdatum_t::middle_center);
  spr.setTextColor(C(160,160,160));
  char buf[24];
  if(avgMPGCnt<1000) snprintf(buf,sizeof(buf),"%d samples",avgMPGCnt);
  else               snprintf(buf,sizeof(buf),"%.1fk samples",avgMPGCnt/1000.0f);
  spr.drawString(buf,120,155);
  modeDots(1,4);
}

static void renderTiming(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  float t=constrain(d.timing,-10.0f,35.0f);
  segArc(t+10.0f,0,45,30,0.2f,0.55f,C(220,60,0),C(230,180,0),C(0,200,80),C(25,15,35));
  uint32_t tc=d.timing>15?TFT_WHITE:d.timing>5?C(230,180,0):C(220,60,0);
  spr.drawCircle(120,120,87,tc);
  drawDialLabels(120,120,74,-10,35,5,"",TFT_WHITE);
  char tb[10];snprintf(tb,sizeof(tb),"%.1f",d.timing);
  glowText(String(tb),108,112,3,tc,C(30,20,0));
  spr.drawCircle(140,95,3,tc);
  glowText("TIMING ADV",120,147,1,C(200,140,0),C(50,30,0));
  modeDots(3,5);
}

static void renderShiftPattern(const CarData& d){
  static float kx=120,ky=120;
  // Gear knob target positions (1-5)
  static const float gx[]={75,75,120,120,168};
  static const float gy[]={78,162,78,162,78};
  float tx=120,ty=120;
  if(d.gear>=1&&d.gear<=5){tx=gx[d.gear-1];ty=gy[d.gear-1];}
  kx+=(tx-kx)*0.35f; ky+=(ty-ky)*0.35f;

  spr.fillScreen(C(5,2,15)); cornerDeco();
  uint32_t rail=C(70,35,10);
  // Horizontal gate
  spr.drawFastHLine(58,120,124,rail);
  // Column verticals
  spr.drawFastVLine(75,78,84,rail);
  spr.drawFastVLine(120,78,84,rail);
  spr.drawFastVLine(168,78,42,rail); // right col only top half (no 6th gear)

  // Gear position indicators
  for(int g=0;g<5;g++){
    bool active=(d.gear==g+1);
    uint32_t bg=active?C(100,20,0):C(15,8,5);
    uint32_t border=active?C(255,80,0):C(50,25,10);
    spr.fillCircle((int)gx[g],(int)gy[g],14,bg);
    spr.drawCircle((int)gx[g],(int)gy[g],14,border);
    if(active){spr.drawCircle((int)gx[g],(int)gy[g],15,C(200,60,0));}
    spr.setTextDatum(textdatum_t::middle_center);spr.setTextSize(2);
    spr.setTextColor(active?TFT_WHITE:C(60,30,10));
    spr.drawString(String(g+1),(int)gx[g],(int)gy[g]);
  }
  // Neutral label on gate
  spr.setTextSize(1);spr.setTextDatum(textdatum_t::middle_center);spr.setTextColor(C(60,35,10));
  spr.drawString("N",120,108);

  // Animated shift knob
  spr.fillCircle((int)kx,(int)ky,9,C(180,90,20));
  spr.drawCircle((int)kx,(int)ky,9,C(255,200,80));
  spr.drawCircle((int)kx,(int)ky,10,C(120,60,10));

  // Info at bottom
  String gs=d.gear>0?"GEAR "+String(d.gear):"NEUTRAL";
  glowText(gs,120,198,2,C(255,140,0),C(70,35,0));
  float mph=d.speed_kph*0.621371f;
  spr.setTextSize(1);spr.setTextDatum(textdatum_t::middle_center);spr.setTextColor(C(80,50,30));
  char sb[28];snprintf(sb,sizeof(sb),"%d mph  %d rpm",(int)mph,(int)d.rpm);
  spr.drawString(sb,120,215);
  modeDots(2,4);
}

static void renderFuelUsed(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  // Instantaneous fuel rate: gal/hr from MAF
  static constexpr float GAL_PER_G_AIR = 1.0f/(14.7f*745.0f*3.78541f);
  float rate=d.maf_g_s*3600.0f*GAL_PER_G_AIR;
  spr.drawCircle(120,120,87,C(0,160,200));
  // Gallons used (large centre)
  char gb[12];
  if(fuelUsed_gal<10.0f) snprintf(gb,sizeof(gb),"%.3f",fuelUsed_gal);
  else                   snprintf(gb,sizeof(gb),"%.2f",fuelUsed_gal);
  glowText(String(gb),120,90,3,TFT_WHITE,C(40,40,40));
  glowText("GAL USED",120,125,1,TFT_WHITE,C(40,40,40));
  spr.drawFastHLine(55,143,130,C(25,30,40));
  // Current burn rate
  char rb[16];snprintf(rb,sizeof(rb),"%.3f gal/hr",rate);
  glowText(String(rb),120,163,1,TFT_WHITE,C(40,40,40));
  glowText("CURRENT RATE",120,180,1,TFT_WHITE,C(40,40,40));
  spr.setTextSize(1);spr.setTextDatum(textdatum_t::middle_center);
  spr.setTextColor(C(60,40,40));
  spr.drawString("hold btn 3s to reset",120,208);
  modeDots(3,4);
}

static void renderOdometer(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  spr.drawCircle(120,120,87,C(0,180,100));
  // Total tracked miles (committed miles + sub-0.1 accumulator)
  char ob[16];
  snprintf(ob, sizeof(ob), "%d", (int)(odomMiles + odomAccum));
  glowText(String(ob),120,82,3,TFT_WHITE,C(40,40,40));
  glowText("MILES TRACKED",120,115,1,TFT_WHITE,C(40,40,40));
  spr.drawFastHLine(50,133,140,C(20,40,28));
  // Trip miles
  char tb[14];
  if(tripMiles<100.0f) snprintf(tb,sizeof(tb),"%.2f mi",tripMiles);
  else                 snprintf(tb,sizeof(tb),"%.1f mi",tripMiles);
  glowText(String(tb),120,155,2,TFT_WHITE,C(40,40,40));
  glowText("THIS TRIP",120,175,1,TFT_WHITE,C(40,40,40));
  spr.setTextSize(1);spr.setTextDatum(textdatum_t::middle_center);spr.setTextColor(C(60,60,60));
  spr.drawString("trip resets on power cycle",120,207);
  modeDots(3,4);
}

static void renderBattery(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  float bp=constrain((d.batt_v-11.0f)/4.0f,0.0f,1.0f);
  segArc(bp*40.0f,0,40,30,0.25f,0.625f,C(220,0,0),C(230,180,0),C(0,200,80),C(25,15,35));
  uint32_t bc=d.batt_v>13.5f?C(0,200,80):d.batt_v>12.4f?C(230,180,0):C(220,0,0);
  spr.drawCircle(120,120,87,bc);
  char bv[10];snprintf(bv,sizeof(bv),"%.2fV",d.batt_v);
  glowText(String(bv),120,100,3,bc,C(20,20,0));
  glowText("BATTERY",120,132,1,C(180,120,0),C(50,30,0));
  const char* st=d.batt_v>13.5f?"CHARGING":d.batt_v>12.4f?"GOOD":"LOW";
  glowText(String(st),120,162,2,bc,C(20,20,0));
  modeDots(3,5);
}

static void bootAnimation(){
  const int carY=103, roadY=162, step=50;
  lgfx::LGFX_Device* disps[3]={&disp1,&disp2,&disp3};
  for(int vx=-(int)COROLLA_W;vx<=720;vx+=step){
    for(int d=0;d<3;d++){
      int localX=vx-d*240;
      spr.fillScreen(C(5,5,10));
      spr.fillRect(0,roadY,240,240-roadY,C(12,12,12));
      spr.drawFastHLine(0,roadY,240,C(65,65,65));
      for(int i=0;i<4;i++){
        int lineEnd=localX-4-i*4,lineLen=22+i*10,lineStart=lineEnd-lineLen,ly=carY+22+i*6;
        int ds=max(0,lineStart),de=min(239,lineEnd);
        if(ds<de) spr.drawFastHLine(ds,ly,de-ds,C(160-i*35,160-i*35,160-i*35));
      }
      draw1BPPBitmap(localX,carY,corolla_bits,COROLLA_W,COROLLA_H,TFT_WHITE,true);
      spr.pushSprite(disps[d],0,0);
    }
    delay(20);
  }
  spr.fillScreen(TFT_BLACK);
  for(int d=0;d<3;d++) spr.pushSprite(disps[d],0,0);
}

static void bootScreen(){
  lgfx::LGFX_Device* disps[3]={&disp1,&disp2,&disp3};
  for(int i=0;i<3;i++){
    spr.fillScreen(TFT_BLACK);spr.setTextDatum(textdatum_t::middle_center);
    spr.setTextColor(C(220,40,0));spr.setTextSize(3);
    spr.drawString("Car",120,95);spr.drawString("Coach",120,135);
    spr.pushSprite(disps[i],0,0);
  }
  delay(2000);
}

void obdTask(void*){
  bool pidProbed=false; (void)pidProbed;
  for(;;){
    if(!obd.connected){
      obd.begin();
      if(!obd.connected){ vTaskDelay(pdMS_TO_TICKS(2000)); continue; }
    }
#if PID_TEST
    if(!pidProbed){ obd.probePIDs(); pidProbed=true; }
#endif
    // Tiered polling: fast-changing PIDs every cycle, slow ones every N cycles
    static int pollCycle=0; pollCycle++;
    uint16_t flags=0x003;                              // RPM + speed every cycle
    if(pollCycle%8==0)     flags|=0x040;              // battery every 8 cycles
    if(mode1==0||mode1==2) flags|=0x008;              // load (changes with driving)
    if(mode1==1||mode3==1||mode3==4) flags|=0x020;   // MAF for MPG/fuel screens
    if(mode1==3)           flags|=0x400;              // timing advance
    if(mode2==1)           flags|=0x010;              // throttle
    if(mode3==0){
      flags|=0x004;                                   // coolant every cycle when on screen
      if(pollCycle%3==0) flags|=0x080;               // IAT every 3 cycles (changes slowly)
    }
    if(mode3==2){
      flags|=0x100;                                   // STFT every cycle when on screen
      if(pollCycle%3==0) flags|=0x200;               // LTFT every 3 cycles (changes very slowly)
    }
    obd.pollAll(flags);

    // Gear detection using RPM/speed ratio matched against known gear ratios
    static constexpr float WK = 1000.0f/60.0f/WHEEL_CIRC_M;
    static int confirmedGear=0,pendingGear=0,pendingCount=0;
    int raw=0;
    if(obd.speed_kph<4.8f||obd.rpm<500){
      raw=0;
    } else {
      float ratio=obd.rpm/obd.speed_kph;
      float minDiff=1e9f;
      for(int g=0;g<NUM_GEARS;g++){
        float expected=GEAR_RATIOS[g]*WK;
        float diff=fabsf(ratio-expected);
        if(diff<minDiff){minDiff=diff;raw=g+1;}
      }
      if(fabsf(ratio-GEAR_RATIOS[raw-1]*WK)/(GEAR_RATIOS[raw-1]*WK)>0.25f) raw=0;
    }
    if(confirmedGear>0&&raw>0&&abs(raw-confirmedGear)>1) raw=0;
    if(raw==pendingGear){if(++pendingCount>=3)confirmedGear=raw;}
    else                {pendingGear=raw;pendingCount=0;}
    int gear=confirmedGear;
    xSemaphoreTake(dataMutex,portMAX_DELAY);
    carData={obd.rpm,obd.speed_kph*SPEED_CORRECTION,obd.coolant_c,obd.load_pct,
             obd.throttle,obd.iat_c,obd.batt_v,obd.maf_g_s,obd.stft,obd.ltft,obd.timing,gear,true};
    xSemaphoreGive(dataMutex);
  }
}

void displayTask(void*){
  static CarData smooth={};
  static bool smoothInit=false;
  for(;;){
    checkButtons();
#if DEMO_MODE
    updateDemo();
#endif
    CarData snap;
    xSemaphoreTake(dataMutex,portMAX_DELAY);snap=carData;xSemaphoreGive(dataMutex);

    // Seed smoother on first valid OBD frame to avoid animating from zero
    if(!smoothInit&&snap.obd_ok){smooth=snap;smoothInit=true;}

    // Lerp display values toward latest OBD data each frame
    const float k=0.35f;
    smooth.rpm       +=( snap.rpm       -smooth.rpm       )*k;
    smooth.speed_kph +=( snap.speed_kph -smooth.speed_kph )*k;
    smooth.load_pct  +=( snap.load_pct  -smooth.load_pct  )*k;
    smooth.coolant_c +=( snap.coolant_c -smooth.coolant_c )*k;
    smooth.throttle  +=( snap.throttle  -smooth.throttle  )*k;
    smooth.maf_g_s   +=( snap.maf_g_s   -smooth.maf_g_s   )*k;
    smooth.batt_v    +=( snap.batt_v    -smooth.batt_v    )*k;
    smooth.stft      +=( snap.stft      -smooth.stft      )*k;
    smooth.ltft      +=( snap.ltft      -smooth.ltft      )*k;
    smooth.timing    +=( snap.timing    -smooth.timing    )*k;
    smooth.gear=snap.gear; smooth.obd_ok=snap.obd_ok;

    updateCenterSceneMotion(smooth.speed_kph);

    bool warnTemp=snap.obd_ok&&snap.coolant_c>WARN_TEMP_C&&snap.rpm>400.0f;
    auto applyAlert=[&](){
      if(warnTemp) drawAlert("OVERHEAT",String((int)snap.coolant_c)+"C",C(220,0,0),TFT_WHITE);
    };

    // Update average MPG and fuel used with raw snap values (not smoothed)
    {
      static uint32_t lastFuelMs=0;
      uint32_t nowMs=millis();
      float mph2=snap.speed_kph*0.621371f;
      if(mph2>5.0f&&snap.maf_g_s>0.1f){
        float inst=constrain(snap.speed_kph*7.103f/snap.maf_g_s,0.0f,80.0f);
        if(inst>0){avgMPG=(avgMPGCnt==0)?inst:avgMPG*0.995f+inst*0.005f;avgMPGCnt++;}
      }
      if(snap.maf_g_s>0.1f&&lastFuelMs>0){
        float dt=(nowMs-lastFuelMs)/1000.0f;
        if(dt>0&&dt<2.0f) fuelUsed_gal+=snap.maf_g_s*dt/(14.7f*745.0f*3.78541f);
      }
      lastFuelMs=nowMs;
      if((fuelUsed_gal-lastSavedFuel>=0.01f) || (nowMs-lastFuelSaveMs>=30000)){
        Preferences p;
        p.begin("carcoach",false);
        p.putFloat("fuel_gal",fuelUsed_gal);
        p.end();
        lastSavedFuel=fuelUsed_gal;
        lastFuelSaveMs=nowMs;
      }
      if(fuelUsed_gal-lastSavedFuel>=0.1f){
        Preferences p;p.begin("carcoach",false);p.putFloat("fuel_gal",fuelUsed_gal);p.end();
        lastSavedFuel=fuelUsed_gal;
      }
    }

    // Odometer accumulation
    {
      static uint32_t lastOdomMs   = 0;
      static float    lastSavedOdom = 0.0f;

      uint32_t nowMs = millis();

      if (snap.obd_ok && snap.speed_kph > 0.5f && lastOdomMs > 0) {
        float dt_hr = (nowMs - lastOdomMs) / 3600000.0f;
        float mph   = snap.speed_kph * 0.621371f;
        float delta = mph * dt_hr;

        if (delta > 0.0f && delta < 0.05f) {
          odomAccum += delta;
          tripMiles += delta;

          while (odomAccum >= 0.1f) {
            odomMiles += 0.1f;
            odomAccum -= 0.1f;
          }
        }
      }

      lastOdomMs = nowMs;

      if (odomMiles - lastSavedOdom >= 0.1f) {
        Preferences p;
        p.begin("carcoach", false);
        p.putFloat("odo_miles", odomMiles);
        p.end();
        lastSavedOdom = odomMiles;
      }
    }

    switch(mode1){case 0:renderRPM(smooth);break;case 1:renderMPG(smooth);break;case 2:renderLoad(smooth);break;case 3:renderTiming(smooth);break;}
    applyAlert(); spr.pushSprite(&disp1,0,0);
    switch(mode2){case 0:renderCorollaScene(smooth);break;case 1:renderThrottle(smooth);break;case 2:renderShiftPattern(smooth);break;case 3:renderOdometer(smooth);break;}
    applyAlert(); spr.pushSprite(&disp2,0,0);
    switch(mode3){
      case 0: renderSystems(smooth);   break;
      case 1: renderAvgMPG(smooth);    break;
      case 2: renderFuelTrim(smooth);  break;
      // case 3: renderBattery(smooth); -- battery polling unreliable, screen hidden
      case 3: renderFuelUsed(smooth);  break;
    }
    applyAlert(); spr.pushSprite(&disp3,0,0);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void setup(){
  Serial.begin(115200);delay(300);
  Serial.println("===== CARCOACH BOOT =====");
  Serial.printf("DEMO_MODE = %d\n",DEMO_MODE);
  pinMode(BTN1_PIN,INPUT_PULLUP);pinMode(BTN2_PIN,INPUT_PULLUP);pinMode(BTN3_PIN,INPUT_PULLUP);
  pinMode(TFT_RST,OUTPUT);digitalWrite(TFT_RST,LOW);delay(20);digitalWrite(TFT_RST,HIGH);delay(150);
  disp1.init();disp1.invertDisplay(true);disp1.setRotation(0);disp1.fillScreen(TFT_BLACK);
  disp2.init();disp2.invertDisplay(true);disp2.setRotation(0);disp2.fillScreen(TFT_BLACK);
  disp3.init();disp3.invertDisplay(true);disp3.setRotation(0);disp3.fillScreen(TFT_BLACK);
  spr.setColorDepth(8);
  if(!spr.createSprite(240,240)){Serial.println("Sprite alloc failed!");while(true)delay(1000);}
  Serial.printf("Free heap: %d bytes\n",ESP.getFreeHeap());
  { Preferences p;p.begin("carcoach",true);
    float saved=p.getFloat("avg_mpg",0.0f);int savedCnt=p.getInt("avg_cnt",0);
    float savedOdo=p.getFloat("odo_miles",0.0f);
    float savedFuel=p.getFloat("fuel_gal",0.0f);
    p.end();
    if(saved>0.0f&&savedCnt>0){
      avgMPG=saved;avgMPGCnt=min(savedCnt,5000);
      Serial.printf("Loaded avg MPG: %.1f (%d samples)\n",avgMPG,avgMPGCnt);
    }
    // Accept NVS value only if it's within 2000 miles above STARTING_ODOMETER;
    // otherwise the saved value is stale/corrupt (e.g. from a bad config flash).
    if(savedOdo >= STARTING_ODOMETER && savedOdo <= STARTING_ODOMETER + 2000.0f)
      odomMiles = savedOdo;
    else
      odomMiles = STARTING_ODOMETER;
    Serial.printf("Loaded odometer: %.1f miles (NVS=%.1f)\n",odomMiles,savedOdo);
    if(savedFuel>0.0f){
      fuelUsed_gal=savedFuel;
      lastSavedFuel=savedFuel;
      lastFuelSaveMs=millis();
      Serial.printf("Loaded fuel used: %.3f gal\n",fuelUsed_gal);
    }
  }
  bootAnimation();
  dataMutex=xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(displayTask,"display",8192,nullptr,1,nullptr,1);
  xTaskCreatePinnedToCore(obdTask,"obd",8192,nullptr,1,nullptr,0);
}
void loop(){vTaskDelay(portMAX_DELAY);}
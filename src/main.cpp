#include <Arduino.h>
#include <LovyanGFX.hpp>
#include "config.h"

template<uint8_t CS_PIN>
class GC9A01 : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel;
  lgfx::Bus_SPI      _bus;
public:
  GC9A01() {
    { auto cfg = _bus.config();
      cfg.spi_host   = VSPI_HOST;
      cfg.freq_write = 27000000;
      cfg.pin_sclk   = TFT_SCK;
      cfg.pin_mosi   = TFT_MOSI;
      cfg.pin_miso   = -1;
      cfg.pin_dc     = TFT_DC;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    { auto cfg = _panel.config();
      cfg.pin_cs        = CS_PIN;
      cfg.pin_rst       = -1;
      cfg.pin_busy      = -1;
      cfg.memory_width  = cfg.panel_width  = 240;
      cfg.memory_height = cfg.panel_height = 240;
      cfg.rgb_order     = false;
      _panel.config(cfg);
    }
    setPanel(&_panel);
  }
};

static GC9A01<TFT1_CS> disp1;
static GC9A01<TFT2_CS> disp2;
static GC9A01<TFT3_CS> disp3;
static LGFX_Sprite spr(&disp1);

static void drawDialLabels(int cx, int cy, int radius,
                           float minVal, float maxVal, int steps,
                           const char* suffix, uint32_t col,
                           float startDeg = 135.0f, float sweepDeg = 270.0f) {
  spr.setTextDatum(textdatum_t::middle_center);
  spr.setTextSize(1);
  spr.setTextColor(col);
  for (int i = 0; i <= steps; i++) {
    float t   = (float)i / steps;
    float val = minVal + (maxVal - minVal) * t;
    float deg = startDeg + sweepDeg * t;
    float rad = deg * DEG_TO_RAD;
    int x = cx + (int)(cosf(rad) * radius);
    int y = cy + (int)(sinf(rad) * radius);
    String s = String((int)val);
    if (suffix && suffix[0]) s += suffix;
    spr.drawString(s, x, y);
  }
}

static SemaphoreHandle_t dataMutex;

struct CarData {
  float rpm, speed_kph, coolant_c, load_pct, throttle, iat_c, batt_v;
  int   gear;
  bool  obd_ok;
} carData;

static volatile uint8_t mode1 = 0;
static volatile uint8_t mode3 = 0;

static uint32_t lp1=0, lp3=0;
static void checkButtons() {
  uint32_t n = millis();
  if (!digitalRead(BTN1_PIN) && n-lp1 > 250) { mode1=(mode1+1)%3; lp1=n; }
  if (!digitalRead(BTN3_PIN) && n-lp3 > 250) { mode3=(mode3+1)%3; lp3=n; }
}

#if DEMO_MODE
static void updateDemo() {
  static uint32_t t0 = millis();
  float t = (millis() - t0) / 1000.0f;
  const float half = 6.0f;
  float phase = fmodf(t, half * 2.0f);
  float mph = (phase < half) ? 80.0f*(phase/half) : 80.0f*(1.0f-((phase-half)/half));
  float kph  = mph * 1.60934f;
  float rpm  = 850.0f + mph * 34.0f + 180.0f * sinf(t * 2.0f);
  float cool = 88.0f + 4.0f * sinf(t * 0.2f);
  float load = 18.0f + mph * 0.7f;
  float thr  = (phase < half) ? (20.0f + mph * 0.75f) : (10.0f + mph * 0.45f);
  float iat  = 32.0f + 2.0f * sinf(t * 0.4f);
  float batt = 14.1f + 0.15f * sinf(t * 0.3f);
  if (rpm  < 800.0f)  rpm  = 800.0f;
  if (load > 100.0f)  load = 100.0f;
  if (thr  > 100.0f)  thr  = 100.0f;
  int gear = 0;
  if (mph > 2.0f) {
    if (mph < 12) gear=1; else if (mph < 22) gear=2;
    else if (mph < 35) gear=3; else if (mph < 52) gear=4; else gear=5;
  }
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  carData = {rpm, kph, cool, load, thr, iat, batt, gear, true};
  xSemaphoreGive(dataMutex);
}
#endif

static uint32_t C(uint8_t r,uint8_t g,uint8_t b){ return spr.color565(r,g,b); }

static void glowText(const String& s, int x, int y, int sz, uint32_t bright, uint32_t glow) {
  spr.setTextSize(sz);
  spr.setTextDatum(textdatum_t::middle_center);
  spr.setTextColor(glow);
  for(int dx=-1;dx<=1;dx++) for(int dy=-1;dy<=1;dy++)
    if(dx||dy) spr.drawString(s,x+dx,y+dy);
  spr.setTextColor(bright);
  spr.drawString(s,x,y);
}

static void segArc(float val,float mn,float mx,int ns,float z1p,float z2p,
                   uint32_t c1,uint32_t c2,uint32_t c3,uint32_t dim,int ro=110,int ri=90){
  float pct=constrain((val-mn)/(mx-mn),0.0f,1.0f);
  int lit=(int)(pct*ns);
  for(int i=0;i<ns;i++){
    float a0=135.0f+(float)i/ns*270.0f+1.2f;
    float a1=135.0f+(float)(i+1)/ns*270.0f-1.2f;
    float sp=(float)i/ns;
    uint32_t cl=(sp<z1p)?c1:(sp<z2p)?c2:c3;
    spr.fillArc(120,120,ro,ri,a0,a1,i<lit?cl:dim);
  }
}

static void cornerDeco(){
  uint32_t c=C(200,40,0);
  spr.drawLine(5,5,20,5,c);    spr.drawLine(5,5,5,20,c);
  spr.drawLine(235,5,220,5,c); spr.drawLine(235,5,235,20,c);
  spr.drawLine(5,235,20,235,c);spr.drawLine(5,235,5,220,c);
  spr.drawLine(235,235,220,235,c);spr.drawLine(235,235,235,220,c);
}
static void modeDots(int cur){
  for(int i=0;i<3;i++)
    spr.fillCircle(110+i*10,225,3,i==cur?C(230,40,0):C(60,20,20));
}

static void renderRPM(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  segArc(d.rpm,0,7000,36,0.57f,0.85f,C(220,0,0),C(230,180,0),C(255,0,180),C(25,15,35));
  spr.fillArc(120,120,115,108,135.0f+0.85f*270.0f,135.0f+270.0f,C(0,255,0));
  spr.drawCircle(120,120,87,C(220,40,0));
  drawDialLabels(120,120,74,0,7,7,"k",TFT_WHITE);
  glowText(String((int)d.rpm),120,100,3,C(230,40,0),C(70,10,0));
  glowText("RPM",120,130,1,C(200,40,0),C(60,0,0));
  String gs=d.gear>0?"GEAR "+String(d.gear):"GEAR N";
  glowText(gs,120,155,2,C(255,140,0),C(80,30,0));
  spr.setTextDatum(textdatum_t::middle_center);
  spr.setTextSize(1); spr.setTextColor(C(100,60,60));
  spr.drawString("LOAD",120,178);
  spr.fillRoundRect(80,184,80,6,3,C(20,20,35));
  spr.fillRoundRect(80,184,(int)(d.load_pct/100.0f*80),6,3,C(220,40,0));
  modeDots(0);
}
static void renderLoad(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  segArc(d.load_pct,0,100,30,0.6f,0.85f,C(220,0,0),C(230,180,0),C(255,0,180),C(25,15,35));
  spr.drawCircle(120,120,87,C(255,140,0));
  glowText(String((int)d.load_pct)+"%",120,100,3,C(255,140,0),C(80,30,0));
  glowText("ENGINE LOAD",120,135,1,C(200,100,0),C(50,20,0));
  modeDots(1);
}
static void renderIAT(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  segArc(d.iat_c,0,80,30,0.5f,0.75f,C(255,40,0),C(230,180,0),C(255,0,180),C(25,15,35));
  spr.drawCircle(120,120,87,C(220,20,0));
  glowText(String((int)d.iat_c),108,100,3,C(255,40,0),C(80,0,0));
  spr.drawCircle(140,87,4,C(255,40,0));
  glowText("C",154,100,3,C(255,40,0),C(80,0,0));
  glowText("INTAKE AIR",120,135,1,C(200,30,0),C(60,0,0));
  modeDots(2);
}

static void renderSpeed(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  float mph = d.speed_kph * 0.621371f;
  segArc(mph,0,120,36,0.5f,0.75f,C(0,180,220),C(0,220,180),C(255,0,180),C(25,15,35));
  spr.drawCircle(120,120,87,C(0,180,220));
  drawDialLabels(120,120,74,0,120,6,"",TFT_WHITE);
  glowText(String((int)mph),120,100,3,C(0,180,220),C(0,40,60));
  glowText("MPH",120,130,1,C(0,150,180),C(0,30,40));
  String gs=d.gear>0?"GEAR "+String(d.gear):"GEAR N";
  glowText(gs,120,155,2,C(255,140,0),C(80,30,0));
}

static void renderSystems(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  uint32_t tc=d.coolant_c<95?C(220,0,0):d.coolant_c<105?C(230,180,0):C(255,0,180);
  segArc(d.coolant_c,40,120,28,0.69f,0.81f,C(220,0,0),C(230,180,0),C(255,0,180),C(25,15,35));
  spr.drawCircle(120,120,87,tc);
  drawDialLabels(120,120,74,40,120,4,"",TFT_WHITE);
  glowText(String((int)d.coolant_c),108,95,3,tc,C(30,30,0));
  spr.drawCircle(140,82,4,tc);
  glowText("C",154,95,3,tc,C(30,30,0));
  glowText("ENGINE TEMP",120,128,1,C(150,100,0),C(30,20,0));
  spr.setTextSize(1); spr.setTextDatum(textdatum_t::middle_center);
  spr.setTextColor(C(180,20,0));
  spr.drawString("BATTERY",120,148);
  float bp=constrain((d.batt_v-11.0f)/4.0f,0,1);
  uint32_t bc=d.batt_v>13.5f?C(220,0,0):d.batt_v>12.0f?C(230,180,0):C(255,0,180);
  spr.fillRoundRect(60,155,120,8,3,C(20,20,35));
  spr.fillRoundRect(60,155,(int)(bp*120),8,3,bc);
  char bv[8]; snprintf(bv,sizeof(bv),"%.2fV",d.batt_v);
  glowText(String(bv),120,174,2,bc,C(20,20,0));
  spr.fillCircle(120,206,4,d.obd_ok?C(220,0,0):C(255,0,180));
  spr.setTextSize(1); spr.setTextColor(C(90,40,40));
  spr.setTextDatum(textdatum_t::middle_center);
  spr.drawString(d.obd_ok?"OBD OK":"NO OBD",120,218);
  modeDots(0);
}
static void renderVoltage(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  segArc(d.batt_v,10,16,30,0.33f,0.5f,C(255,0,180),C(230,180,0),C(220,0,0),C(25,15,35));
  uint32_t vc=d.batt_v>13.0f?C(220,0,0):d.batt_v>12.0f?C(230,180,0):C(255,0,180);
  spr.drawCircle(120,120,87,vc);
  char bv[10]; snprintf(bv,sizeof(bv),"%.2fV",d.batt_v);
  glowText(String(bv),120,100,3,vc,C(20,20,0));
  glowText("BATTERY",120,135,1,C(200,40,0),C(60,0,0));
  modeDots(1);
}
static void renderRaw(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  uint32_t gc=C(220,0,0),dc=C(80,0,0);
  spr.setTextSize(1);
  struct Row{const char* lbl;String val;};
  Row rows[]={{"RPM  ",String((int)d.rpm)},{"SPD  ",String((int)d.speed_kph)+" k"},
    {"COOL ",String((int)d.coolant_c)+" C"},{"LOAD ",String((int)d.load_pct)+"%"},
    {"THR  ",String((int)d.throttle)+"%"},{"IAT  ",String((int)d.iat_c)+" C"},
    {"BATT ",String(d.batt_v,2)+"V"}};
  int y=36;
  for(auto& r:rows){
    spr.setTextColor(dc); spr.setTextDatum(textdatum_t::middle_left);  spr.drawString(r.lbl,28,y);
    spr.setTextColor(gc); spr.setTextDatum(textdatum_t::middle_right); spr.drawString(r.val,212,y);
    y+=22;
  }
  modeDots(2);
}

static void bootScreen() {
  lgfx::LGFX_Device* disps[3] = { &disp1, &disp2, &disp3 };
  for (int i = 0; i < 3; i++) {
    spr.fillScreen(TFT_BLACK);
    spr.setTextDatum(textdatum_t::middle_center);
    spr.setTextColor(C(220, 40, 0));
    spr.setTextSize(3);
    spr.drawString("Car", 120, 95);
    spr.drawString("Coach", 120, 135);
    spr.pushSprite(disps[i], 0, 0);
  }
  delay(2000);
}

void displayTask(void*){
  for(;;){
    checkButtons();
#if DEMO_MODE
    updateDemo();
#endif
    CarData snap;
    xSemaphoreTake(dataMutex,portMAX_DELAY); snap=carData; xSemaphoreGive(dataMutex);

    switch(mode1){ case 0: renderRPM(snap);  break; case 1: renderLoad(snap); break; case 2: renderIAT(snap); break; }
    spr.pushSprite(&disp1,0,0);
    renderSpeed(snap);
    spr.pushSprite(&disp2,0,0);
    switch(mode3){ case 0: renderSystems(snap); break; case 1: renderVoltage(snap); break; case 2: renderRaw(snap); break; }
    spr.pushSprite(&disp3,0,0);
    vTaskDelay(pdMS_TO_TICKS(33));
  }
}

void setup(){
  Serial.begin(115200); delay(300);
  Serial.println("===== CARCOACH BOOT =====");
  Serial.printf("DEMO_MODE = %d\n", DEMO_MODE);
  pinMode(BTN1_PIN,INPUT_PULLUP); pinMode(BTN2_PIN,INPUT_PULLUP); pinMode(BTN3_PIN,INPUT_PULLUP);
  pinMode(TFT_RST,OUTPUT); digitalWrite(TFT_RST,LOW); delay(20); digitalWrite(TFT_RST,HIGH); delay(150);
  disp1.init(); disp1.invertDisplay(true); disp1.setRotation(0); disp1.fillScreen(TFT_BLACK);
  disp2.init(); disp2.invertDisplay(true); disp2.setRotation(0); disp2.fillScreen(TFT_BLACK);
  disp3.init(); disp3.invertDisplay(true); disp3.setRotation(0); disp3.fillScreen(TFT_BLACK);
  spr.setColorDepth(8);
  if(!spr.createSprite(240,240)){ Serial.println("Sprite alloc failed!"); while(true) delay(1000); }
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  bootScreen();
  dataMutex=xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(displayTask,"display",8192,nullptr,1,nullptr,1);
}
void loop(){ vTaskDelay(portMAX_DELAY); }

#include <Arduino.h>
#include <LovyanGFX.hpp>
#include "config.h"
#include "obd.h"
#include "corolla_bitmap.h"

// ═════════════════════════════════════════════════════════════════════
//  DISPLAY SETUP
// ═════════════════════════════════════════════════════════════════════

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

// Sprite attached to disp1 — draw here, push to whichever display needed.
static LGFX_Sprite spr(&disp1);

static void drawDialLabels(int cx, int cy, int radius,
                           float minVal, float maxVal, int steps,
                           const char* suffix, uint32_t col,
                           float startDeg = 135.0f, float sweepDeg = 270.0f) {
  spr.setTextDatum(textdatum_t::middle_center);
  spr.setTextSize(1);
  spr.setTextColor(col);

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
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
// ═════════════════════════════════════════════════════════════════════
//  SHARED STATE
// ═════════════════════════════════════════════════════════════════════

static OBDReader         obd;
static SemaphoreHandle_t dataMutex;

struct CarData {
  float rpm, speed_kph, coolant_c, load_pct, throttle, iat_c, batt_v, maf_g_s;
  int   gear;
  bool  obd_ok;
} carData;

static volatile uint8_t mode1 = 0;  // 0=RPM  1=Load  2=MPG
static volatile uint8_t mode2 = 0;  // 0=Corolla scene  1=Throttle
static volatile uint8_t mode3 = 0;  // 0=Coolant+Batt  1=Voltage  2=AvgMPG

static constexpr float WARN_TEMP_C = 105.0f;  // coolant overheat warning
static constexpr float WARN_RPM    = 6000.0f; // high-RPM warning (near redline)

// ═════════════════════════════════════════════════════════════════════
//  BUTTONS
// ═════════════════════════════════════════════════════════════════════

static uint32_t lp1=0, lp2=0, lp3=0;

static void checkButtons() {
  static uint32_t lastPrint = 0;
if (millis() - lastPrint > 500) {
  Serial.printf("BTN1:%d BTN2:%d BTN3:%d modes: %d %d %d\n",
                digitalRead(BTN1_PIN),
                digitalRead(BTN2_PIN),
                digitalRead(BTN3_PIN),
                mode1, mode2, mode3);
  lastPrint = millis();
}
  uint32_t n = millis();
  if(!digitalRead(BTN1_PIN) && n-lp1 > 250){ mode1=(mode1+1)%3; lp1=n; }

  // BTN2 reserved for later scene switching
  if(!digitalRead(BTN2_PIN) && n-lp2 > 250){ mode2=(mode2+1)%2; lp2=n; }

  if(!digitalRead(BTN3_PIN) && n-lp3 > 250){ mode3=(mode3+1)%3; lp3=n; }
}

// ═════════════════════════════════════════════════════════════════════
//  DEMO MODE
// ═════════════════════════════════════════════════════════════════════
#if DEMO_MODE
static void updateDemo() {
  static uint32_t t0 = millis();
  float t = (millis() - t0) / 1000.0f;

  // 0 -> 80 mph in 6 sec, then 80 -> 0 mph in 6 sec
  const float halfCycle = 6.0f;
  float phase = fmodf(t, halfCycle * 2.0f);

  float mph;
  if (phase < halfCycle) {
    mph = 80.0f * (phase / halfCycle);
  } else {
    mph = 80.0f * (1.0f - ((phase - halfCycle) / halfCycle));
  }

  float kph = mph * 1.60934f;

  float rpm  = 850.0f + mph * 34.0f + 180.0f * sinf(t * 2.0f);
  float cool = 88.0f + 4.0f * sinf(t * 0.2f);
  float load = 18.0f + mph * 0.7f;
  float thr  = (phase < halfCycle) ? (20.0f + mph * 0.75f) : (10.0f + mph * 0.45f);
  float iat  = 32.0f + 2.0f * sinf(t * 0.4f);
  float batt = 14.1f + 0.15f * sinf(t * 0.3f);

  if (rpm < 800.0f) rpm = 800.0f;
  if (load > 100.0f) load = 100.0f;
  if (thr > 100.0f) thr = 100.0f;

  int gear = 0;
  if (mph > 2.0f) {
    if      (mph < 12) gear = 1;
    else if (mph < 22) gear = 2;
    else if (mph < 35) gear = 3;
    else if (mph < 52) gear = 4;
    else               gear = 5;
  }

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  carData = {rpm, kph, cool, load, thr, iat, batt, 0.0f, gear, true};
  xSemaphoreGive(dataMutex);
}
#endif

// ═════════════════════════════════════════════════════════════════════
//  DRAWING HELPERS
// ═════════════════════════════════════════════════════════════════════

static uint32_t C(uint8_t r,uint8_t g,uint8_t b){
  return spr.color565(r,g,b);
}

static void glowText(const String& s, int x, int y, int sz,
                     uint32_t bright, uint32_t glow) {
  spr.setTextSize(sz);
  spr.setTextDatum(textdatum_t::middle_center);
  spr.setTextColor(glow);
  for(int dx=-1;dx<=1;dx++) for(int dy=-1;dy<=1;dy++)
    if(dx||dy) spr.drawString(s,x+dx,y+dy);
  spr.setTextColor(bright);
  spr.drawString(s,x,y);
}

static void segArc(float val,float mn,float mx,int ns,
                   float z1p,float z2p,
                   uint32_t c1,uint32_t c2,uint32_t c3,uint32_t dim,
                   int ro=110,int ri=90) {
  float pct=constrain((val-mn)/(mx-mn),0.0f,1.0f);
  int lit=(int)(pct*ns);
  for(int i=0;i<ns;i++){
    float a0=135.0f+(float)i    /ns*270.0f+1.2f;
    float a1=135.0f+(float)(i+1)/ns*270.0f-1.2f;
    float sp=(float)i/ns;
    uint32_t cl=(sp<z1p)?c1:(sp<z2p)?c2:c3;
    spr.fillArc(120,120,ro,ri,a0,a1,i<lit?cl:dim);
  }
}

static void synthGrid(int hy,uint32_t nc,uint32_t fc){
  for(int i=0;i<7;i++){
    float t=(float)(i+1)/7;
    int y=hy+(int)((240-hy)*t*t);
    uint8_t r=((nc>>11)&0x1F)*8*(1-t)+((fc>>11)&0x1F)*8*t;
    uint8_t g=((nc>>5)&0x3F)*4*(1-t)+((fc>>5)&0x3F)*4*t;
    uint8_t b=((nc)&0x1F)*8*(1-t)+((fc)&0x1F)*8*t;
    spr.drawLine(0,y,240,y,spr.color565(r,g,b));
  }
  for(int i=0;i<=12;i++) spr.drawLine(120,hy,i*20,240,nc);
}

static void cornerDeco(){
  uint32_t c=C(200,40,0);
  spr.drawLine(5,5,20,5,c);   spr.drawLine(5,5,5,20,c);
  spr.drawLine(235,5,220,5,c);spr.drawLine(235,5,235,20,c);
  spr.drawLine(5,235,20,235,c);spr.drawLine(5,235,5,220,c);
  spr.drawLine(235,235,220,235,c);spr.drawLine(235,235,235,220,c);
}

static void modeDots(int cur){
  for(int i=0;i<3;i++)
    spr.fillCircle(110+i*10,225,3,i==cur?C(230,40,0):C(60,20,20));
}

// ═════════════════════════════════════════════════════════════════════
//  CENTER DISPLAY SCENE ANIMATION
// ═════════════════════════════════════════════════════════════════════

static float laneShift = 0.0f;
static uint32_t lastSceneTick = 0;

static void updateCenterSceneMotion(float speed_kph){
  uint32_t now = millis();
  float dt = (lastSceneTick == 0) ? 0.033f : (now - lastSceneTick) / 1000.0f;
  lastSceneTick = now;

  float mph = speed_kph * 0.621371f;

  // moving car => vertical lines slide left faster as speed increases
  laneShift -= mph * 5.0f * dt;

  // wrap so it doesn't grow forever
  while (laneShift < -30.0f) laneShift += 30.0f;
  while (laneShift >  30.0f) laneShift -= 30.0f;
}

struct Pt {
  int16_t x;
  int16_t y;
};

template<size_t N>
static void drawPolyline(const Pt (&pts)[N], int ox, int oy, float s, uint32_t col) {
  for(size_t i=0; i+1<N; ++i){
    int x1 = ox + (int)(pts[i].x   * s);
    int y1 = oy + (int)(pts[i].y   * s);
    int x2 = ox + (int)(pts[i+1].x * s);
    int y2 = oy + (int)(pts[i+1].y * s);
    spr.drawLine(x1,y1,x2,y2,col);
  }
}

static void drawPerspectiveGrid(int horizonY){
  // red horizon line
  spr.drawFastHLine(0, horizonY, 240, C(255, 0, 0));

  // horizontal lines
  for (int i = 1; i <= 6; i++) {
    float t = (float)i / 6.0f;
    int y = horizonY + (int)((239 - horizonY) * t * t);
    uint8_t g = (uint8_t)(40 - t * 20.0f);
    if ((int)g < 0) g = 0;
    spr.drawFastHLine(0, y, 240, C(255, g, 0));
  }

  // perspective vertical lines — bottom slides, top converges toward horizon center
  float spacing = 30.0f;
  float offset = fmodf(laneShift, spacing);
  if (offset > 0) offset -= spacing;

  for (float bx = offset; bx < 240.0f + spacing; bx += spacing) {
    // bottom point slides with offset
    // top point is pulled toward horizon center (120) by perspective factor
    float tx = 120.0f + (bx - 120.0f) * 0.25f;
    spr.drawLine((int)bx, 239, (int)tx, horizonY, C(255, 0, 0));
  }
}

static void drawCorollaOutline(int cx, int cy, float s, uint32_t col){
  // Outer body outline
  static const Pt outer[] = {
    {-92, 18}, {-88, 10}, {-76,  5}, {-60,  2}, {-42,  0}, {-22, -3},
    {  6, -5}, { 34, -5}, { 55, -1}, { 74,  7}, { 86, 13}, { 92, 19},
    { 92, 31}, { 88, 36}, { 76, 39}, { 60, 39}, { 54, 50}, { 42, 50},
    { 40, 39}, {-40, 39}, {-42, 50}, {-54, 50}, {-60, 39}, {-76, 39},
    {-88, 36}, {-92, 29}, {-92, 18}
  };

  // Roof / greenhouse
  static const Pt roof[] = {
    {-37, -2}, {-26, -17}, {-12, -24}, { 20, -24}, { 38, -15}, { 54, -6}
  };

  // Window lower line
  static const Pt glassLower[] = {
    {-34, -2}, {-2, -2}, {32, -2}
  };

  // Front windshield line
  static const Pt frontGlass[] = {
    {-10, -22}, {-4, -2}
  };

  // Rear windshield line
  static const Pt rearGlass[] = {
    {22, -22}, {28, -2}
  };

  // Side character line
  static const Pt sideLine[] = {
    {-73, 8}, {-20, 8}, {18, 8}, {66, 11}
  };

  // Rocker panel
  static const Pt rocker[] = {
    {-64, 33}, {-8, 33}, {44, 33}
  };

  // Door seams
  static const Pt door1[] = {
    {-18, -1}, {-17, 39}
  };
  static const Pt door2[] = {
    { 15, -1}, { 16, 39}
  };

  // Door handles
  static const Pt handle1[] = {
    {-12, 12}, {-4, 12}
  };
  static const Pt handle2[] = {
    { 16, 12}, {24, 12}
  };

  // Trunk / rear details
  static const Pt trunkCut[] = {
    {76,  8}, {82, 16}, {82, 31}
  };

  // Headlight hint
  static const Pt headLight[] = {
    {-80, 10}, {-70, 8}, {-66, 14}
  };

  // Fuel door
  int fx = cx + (int)(58 * s);
  int fy = cy + (int)(14 * s);
  int fw = (int)(11 * s);
  int fh = (int)(9 * s);

  drawPolyline(outer,      cx, cy, s, col);
  drawPolyline(roof,       cx, cy, s, col);
  drawPolyline(glassLower, cx, cy, s, col);
  drawPolyline(frontGlass, cx, cy, s, col);
  drawPolyline(rearGlass,  cx, cy, s, col);
  drawPolyline(sideLine,   cx, cy, s, col);
  drawPolyline(rocker,     cx, cy, s, col);
  drawPolyline(door1,      cx, cy, s, col);
  drawPolyline(door2,      cx, cy, s, col);
  drawPolyline(handle1,    cx, cy, s, col);
  drawPolyline(handle2,    cx, cy, s, col);
  drawPolyline(trunkCut,   cx, cy, s, col);
  drawPolyline(headLight,  cx, cy, s, col);

  spr.drawRect(fx, fy, fw, fh, col);

  // mirror hint
  spr.drawLine(cx + (int)(-36*s), cy + (int)(2*s),
               cx + (int)(-30*s), cy + (int)(7*s), col);
  spr.drawLine(cx + (int)(-30*s), cy + (int)(7*s),
               cx + (int)(-24*s), cy + (int)(6*s), col);

  // wheels
  int w1x = cx + (int)(-50*s);
  int w2x = cx + (int)( 50*s);
  int wy  = cy + (int)( 39*s);

  spr.drawCircle(w1x, wy, (int)(18*s), col);
  spr.drawCircle(w1x, wy, (int)(12*s), col);

  spr.drawCircle(w2x, wy, (int)(18*s), col);
  spr.drawCircle(w2x, wy, (int)(12*s), col);
}
static void draw1BPPBitmap(int x, int y, const uint8_t* data, int w, int h, uint32_t color, bool flipX = false) {
  int bytesPerRow = (w + 7) / 8;

  for (int yy = 0; yy < h; yy++) {
    for (int xb = 0; xb < bytesPerRow; xb++) {
      uint8_t b = pgm_read_byte(&data[yy * bytesPerRow + xb]);
      if (!b) continue;

      for (int bit = 0; bit < 8; bit++) {
        int xx = xb * 8 + bit;
        if (xx < w && (b & (1 << bit))) {
          int drawX = flipX ? (x + (w - 1 - xx)) : (x + xx);
          spr.drawPixel(drawX, y + yy, color);
        }
      }
    }
  }
}

static void drawAlert(const String& line1, const String& line2, uint32_t bg, uint32_t fg) {
  spr.fillCircle(120, 120, 85, bg);
  spr.setTextDatum(textdatum_t::middle_center);
  spr.setTextColor(fg);
  spr.setTextSize(2);
  spr.drawString(line1, 120, 100);
  spr.drawString(line2, 120, 138);
}

static void renderCorollaScene(const CarData& d){
  spr.fillScreen(TFT_BLACK);

  const int horizonY = 160;

  // draw ground first
  drawPerspectiveGrid(horizonY);

  // display MPH for the demo / center scene
  float mph = d.speed_kph * 0.621371f;

  // speed in the TOP 1/3 of the 240px screen
  spr.setTextDatum(textdatum_t::middle_center);
  spr.setTextSize(4);
  spr.setTextColor(TFT_WHITE);
  spr.drawString(String((int)roundf(mph)), 120, 60);

  // draw the actual Corolla bitmap centered on the horizon
  int carX = (240 - COROLLA_W) / 2;
  int carY = horizonY - COROLLA_H + 2;

  draw1BPPBitmap(carX, carY, corolla_bits, COROLLA_W, COROLLA_H, TFT_WHITE, true);
}

// ═════════════════════════════════════════════════════════════════════
//  RENDER FUNCTIONS — Display 1
// ═════════════════════════════════════════════════════════════════════

static void renderRPM(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  segArc(d.rpm,0,7000,36,0.57f,0.85f,
         C(220,0,0),C(230,180,0),C(255,0,180),C(25,15,35));
  spr.fillArc(120,120,115,108,135.0f+0.85f*270.0f,135.0f+270.0f,C(0,255,0));
  spr.drawCircle(120,120,87,C(220,40,0));
  drawDialLabels(120, 120, 74, 0, 7, 7, "k", TFT_WHITE);
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
  segArc(d.load_pct,0,100,30,0.6f,0.85f,
         C(220,0,0),C(230,180,0),C(255,0,180),C(25,15,35));
  spr.drawCircle(120,120,87,C(255,140,0));
  glowText(String((int)d.load_pct)+"%",120,100,3,C(255,140,0),C(80,30,0));
  glowText("ENGINE LOAD",120,135,1,C(200,100,0),C(50,20,0));
  modeDots(1);
}

static void renderMPG(const CarData& d) {
  spr.fillScreen(C(5, 2, 15));
  cornerDeco();

  float mph = d.speed_kph * 0.621371f;
  float mpg = (mph > 1.0f && d.maf_g_s > 0.1f)
              ? constrain(d.speed_kph * 7.103f / d.maf_g_s, 0.0f, 99.9f)
              : 0.0f;

  segArc(mpg, 0, 50, 30, 0.2f, 0.6f,
         C(220, 0, 0), C(230, 180, 0), C(0, 200, 80), C(25, 15, 35));
  spr.drawCircle(120, 120, 87, C(0, 190, 60));
  drawDialLabels(120, 120, 74, 0, 50, 5, "", TFT_WHITE);

  if (mph < 1.0f || d.maf_g_s < 0.1f) {
    glowText("--", 120, 100, 3, C(80, 80, 80), C(20, 20, 20));
  } else {
    glowText(String((int)mpg), 120, 100, 3, C(0, 210, 80), C(0, 55, 20));
  }
  glowText("MPG", 120, 130, 1, C(0, 180, 60), C(0, 40, 15));
  modeDots(2);
}

// ═════════════════════════════════════════════════════════════════════
//  RENDER FUNCTIONS — Display 3
// ═════════════════════════════════════════════════════════════════════

static void renderSystems(const CarData& d){
  spr.fillScreen(C(5,2,15)); cornerDeco();
  uint32_t tc=d.coolant_c<95?C(220,0,0):d.coolant_c<105?C(230,180,0):C(255,0,180);
  segArc(d.coolant_c,40,120,28,0.69f,0.81f,
         C(220,0,0),C(230,180,0),C(255,0,180),C(25,15,35));
  spr.drawCircle(120,120,87,tc);
  drawDialLabels(120, 120, 74, 40, 120, 4, "", TFT_WHITE);
  String temp = String((int)d.coolant_c);
  glowText(temp, 108, 95, 3, tc, C(30,30,0));
  spr.drawCircle(140, 82, 4, tc);
  glowText("C", 154, 95, 3, tc, C(30,30,0));

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
  segArc(d.batt_v,10,16,30,0.33f,0.5f,
         C(255,0,180),C(230,180,0),C(220,0,0),C(25,15,35));
  uint32_t vc=d.batt_v>13.0f?C(220,0,0):d.batt_v>12.0f?C(230,180,0):C(255,0,180);
  spr.drawCircle(120,120,87,vc);
  char bv[10]; snprintf(bv,sizeof(bv),"%.2fV",d.batt_v);
  glowText(String(bv),120,100,3,vc,C(20,20,0));
  glowText("BATTERY",120,135,1,C(200,40,0),C(60,0,0));
  modeDots(1);
}

static void renderThrottle(const CarData& d) {
  spr.fillScreen(C(5, 2, 15));
  cornerDeco();

  segArc(d.throttle, 0, 100, 30, 0.5f, 0.8f,
         C(0, 180, 220), C(230, 180, 0), C(220, 0, 0), C(25, 15, 35));
  spr.drawCircle(120, 120, 87, C(0, 160, 200));
  drawDialLabels(120, 120, 74, 0, 100, 5, "", TFT_WHITE);
  glowText(String((int)d.throttle) + "%", 120, 100, 3, C(0, 180, 220), C(0, 40, 60));
  glowText("THROTTLE", 120, 135, 1, C(0, 150, 190), C(0, 30, 50));
  modeDots(1);
}

static float avgMPG    = 0.0f;
static int   avgMPGCnt = 0;

static void renderAvgMPG(const CarData& d) {
  spr.fillScreen(C(5, 2, 15));
  cornerDeco();

  float display = constrain(avgMPG, 0.0f, 50.0f);
  segArc(display, 0, 50, 30, 0.2f, 0.6f,
         C(220, 0, 0), C(230, 180, 0), C(0, 200, 80), C(25, 15, 35));
  spr.drawCircle(120, 120, 87, C(0, 180, 60));
  drawDialLabels(120, 120, 74, 0, 50, 5, "", TFT_WHITE);

  if (avgMPGCnt == 0) {
    glowText("--", 120, 95, 3, C(80, 80, 80), C(20, 20, 20));
  } else {
    glowText(String((int)avgMPG), 120, 95, 3, C(0, 200, 80), C(0, 50, 20));
  }
  glowText("AVG MPG", 120, 130, 1, C(0, 160, 50), C(0, 35, 15));

  spr.setTextSize(1);
  spr.setTextDatum(textdatum_t::middle_center);
  spr.setTextColor(C(50, 80, 50));
  char buf[24];
  snprintf(buf, sizeof(buf), "%d samples", avgMPGCnt);
  spr.drawString(buf, 120, 155);

  modeDots(2);
}

// ═════════════════════════════════════════════════════════════════════
//  BOOT SCREEN
// ═════════════════════════════════════════════════════════════════════

static void bootAnimation() {
  const int carY  = 103;   // vertical position matches the in-game scene
  const int roadY = 162;   // road surface line
  const int step  = 50;    // pixels per frame

  lgfx::LGFX_Device* disps[3] = { &disp1, &disp2, &disp3 };

  for (int vx = -(int)COROLLA_W; vx <= 720; vx += step) {
    for (int d = 0; d < 3; d++) {
      int localX = vx - d * 240;

      spr.fillScreen(C(5, 5, 10));

      // Road surface
      spr.fillRect(0, roadY, 240, 240 - roadY, C(12, 12, 12));
      spr.drawFastHLine(0, roadY, 240, C(65, 65, 65));

      // Speed lines trailing behind the car
      for (int i = 0; i < 4; i++) {
        int lineEnd   = localX - 4 - i * 4;
        int lineLen   = 22 + i * 10;
        int lineStart = lineEnd - lineLen;
        int ly        = carY + 22 + i * 6;
        int ds = max(0, lineStart), de = min(239, lineEnd);
        if (ds < de)
          spr.drawFastHLine(ds, ly, de - ds, C(160 - i*35, 160 - i*35, 160 - i*35));
      }

      // Corolla bitmap — drawPixel clips automatically at sprite edges
      draw1BPPBitmap(localX, carY, corolla_bits, COROLLA_W, COROLLA_H, TFT_WHITE, true);

      spr.pushSprite(disps[d], 0, 0);
    }
    delay(20);
  }

  // Clear all three screens after animation completes
  spr.fillScreen(TFT_BLACK);
  for (int d = 0; d < 3; d++) spr.pushSprite(disps[d], 0, 0);
}

// ═════════════════════════════════════════════════════════════════════
//  RTOS TASKS
// ═════════════════════════════════════════════════════════════════════

void obdTask(void*){
  for(;;){
    if(!obd.connected){
      obd.begin();
      if(!obd.connected){ vTaskDelay(pdMS_TO_TICKS(2000)); continue; }
    }
    obd.pollAll();

    static int confirmedGear = 0;
    static int pendingGear   = 0;
    static int pendingCount  = 0;

    float mph = obd.speed_kph * 0.621371f;
    int raw = 0;

    if(mph < 3.0f){
      raw = 0;                                   // stopped
    } else if(obd.rpm < 1000 && mph > 10.0f){
      raw = 0;                                   // neutral / clutch in while rolling
    } else if(mph < 22.0f){ raw = 1; }
    else if(mph < 32.0f){   raw = 2; }
    else if(mph < 46.0f){   raw = 3; }
    else if(mph < 62.0f){   raw = 4; }
    else{                   raw = 5; }

    if(raw == pendingGear){ if(++pendingCount >= 2) confirmedGear = raw; }
    else                  { pendingGear = raw; pendingCount = 0; }
    int gear = confirmedGear;
    xSemaphoreTake(dataMutex,portMAX_DELAY);
    carData={obd.rpm,obd.speed_kph,obd.coolant_c,obd.load_pct,
             obd.throttle,obd.iat_c,obd.batt_v,obd.maf_g_s,gear,true};
    xSemaphoreGive(dataMutex);
  }
}

void displayTask(void*){
  for(;;){
    checkButtons();

#if DEMO_MODE
    updateDemo();
#endif

    CarData snap;
    xSemaphoreTake(dataMutex,portMAX_DELAY);
    snap=carData;
    xSemaphoreGive(dataMutex);

    updateCenterSceneMotion(snap.speed_kph);

    // ── Alert conditions ──────────────────────────────────────────────
    bool warnTemp = snap.obd_ok && snap.coolant_c > WARN_TEMP_C && snap.rpm > 400.0f;
    bool warnRPM  = snap.obd_ok && snap.rpm > WARN_RPM;

    auto applyAlert = [&]() {
      if      (warnRPM)  drawAlert("HIGH RPM", String((int)snap.rpm) + " rpm", C(200,80,0), TFT_WHITE);
      else if (warnTemp) drawAlert("OVERHEAT", String((int)snap.coolant_c) + "C", C(220,0,0), TFT_WHITE);
    };

    // Update average MPG while driving
    {
      float mph2 = snap.speed_kph * 0.621371f;
      if (mph2 > 5.0f && snap.maf_g_s > 0.1f) {
        float inst = constrain(snap.speed_kph * 7.103f / snap.maf_g_s, 0.0f, 80.0f);
        if (inst > 0) {
          avgMPG = (avgMPGCnt == 0) ? inst : avgMPG * 0.995f + inst * 0.005f;
          avgMPGCnt++;
        }
      }
    }

    // Display 1
    switch(mode1){
      case 0: renderRPM(snap);  break;
      case 1: renderLoad(snap); break;
      case 2: renderMPG(snap);  break;
    }
    applyAlert();
    spr.pushSprite(&disp1,0,0);

    // Display 2
    switch(mode2){
      case 0: renderCorollaScene(snap); break;
      case 1: renderThrottle(snap);     break;
    }
    applyAlert();
    spr.pushSprite(&disp2,0,0);

    // Display 3
    switch(mode3){
      case 0: renderSystems(snap);  break;
      case 1: renderVoltage(snap);  break;
      case 2: renderAvgMPG(snap);   break;
    }
    applyAlert();
    spr.pushSprite(&disp3,0,0);

    vTaskDelay(pdMS_TO_TICKS(33));
  }
}

// ═════════════════════════════════════════════════════════════════════
//  SETUP & LOOP
// ═════════════════════════════════════════════════════════════════════

void setup(){
  Serial.begin(115200);

  delay(300);
Serial.println();
Serial.println("===== CARCOACH BOOT =====");
Serial.printf("DEMO_MODE = %d\n", DEMO_MODE);

  pinMode(BTN1_PIN,INPUT_PULLUP);
  pinMode(BTN2_PIN,INPUT_PULLUP);
  pinMode(BTN3_PIN,INPUT_PULLUP);

  // Reset all 3 displays at once via shared RST pin
  pinMode(TFT_RST,OUTPUT);
  digitalWrite(TFT_RST,LOW);  delay(20);
  digitalWrite(TFT_RST,HIGH); delay(150);

  // Init each display
  disp1.init(); disp1.invertDisplay(true); disp1.setRotation(0); disp1.fillScreen(TFT_BLACK);
  disp2.init(); disp2.invertDisplay(true); disp2.setRotation(0); disp2.fillScreen(TFT_BLACK);
  disp3.init(); disp3.invertDisplay(true); disp3.setRotation(0); disp3.fillScreen(TFT_BLACK);

  // Sprite setup
  spr.setColorDepth(8);

  if(!spr.createSprite(240,240)){
    Serial.println("Sprite alloc failed!");
    disp1.setTextColor(TFT_RED,TFT_BLACK);
    disp1.setCursor(20,110); disp1.print("Sprite alloc failed");
    while(true) delay(1000);
  }

  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());

  bootAnimation();

  dataMutex=xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(displayTask,"display",8192,nullptr,1,nullptr,1);
#if !DEMO_MODE
  xTaskCreatePinnedToCore(obdTask,"obd",8192,nullptr,1,nullptr,0);
#endif
}

void loop(){ vTaskDelay(portMAX_DELAY); }
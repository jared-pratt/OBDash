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

static uint32_t C(uint8_t r, uint8_t g, uint8_t b) {
  return spr.color565(r, g, b);
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

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("===== CARCOACH BOOT =====");

  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);

  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, LOW);  delay(20);
  digitalWrite(TFT_RST, HIGH); delay(150);

  disp1.init(); disp1.invertDisplay(true); disp1.setRotation(0); disp1.fillScreen(TFT_BLACK);
  disp2.init(); disp2.invertDisplay(true); disp2.setRotation(0); disp2.fillScreen(TFT_BLACK);
  disp3.init(); disp3.invertDisplay(true); disp3.setRotation(0); disp3.fillScreen(TFT_BLACK);

  spr.setColorDepth(8);
  if (!spr.createSprite(240, 240)) {
    Serial.println("Sprite alloc failed!");
    while (true) delay(1000);
  }

  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  bootScreen();
}

void loop() { delay(1000); }

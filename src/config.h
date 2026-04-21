#pragma once

// ── Shared SPI bus ───────────────────────────────────────────────────
#define TFT_SCK     18
#define TFT_MOSI    23
#define TFT_DC       2
#define TFT_RST      4

// ── Chip-select per display ──────────────────────────────────────────
#define TFT1_CS     15   // LEFT  — RPM
#define TFT2_CS     13   // CENTER — Speed/scene
#define TFT3_CS     25   // RIGHT  — Engine systems

// ── Cycle buttons (active LOW, internal pullup) ──────────────────────
#define BTN1_PIN    32
#define BTN2_PIN    33
#define BTN3_PIN    27

// ── Set DEMO_MODE 1 to run without a car ────────────────────────────
#define DEMO_MODE   0

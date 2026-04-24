#pragma once

// ── Shared SPI bus ───────────────────────────────────────────────────
#define TFT_SCK     18
#define TFT_MOSI    23
#define TFT_DC       2
#define TFT_RST      4

// ── Chip-select per display ──────────────────────────────────────────
#define TFT1_CS     15
#define TFT2_CS     13
#define TFT3_CS     25

// ── Cycle buttons ────────────────────────────────────────────────────
#define BTN1_PIN    33
#define BTN2_PIN    32
#define BTN3_PIN    27

// ── Set DEMO_MODE 1 to run without a car ────────────────────────────
#define DEMO_MODE   0

// ── 2006 Toyota Corolla CE — C60 MT gear ratios ──────────────────────
// Stock 185/65/R15 → 1952 mm circ. Actual 215/45/R17 → 1964 mm circ.
// Larger tires rotate fewer times/km so ECU under-reads; multiply to correct.
#define SPEED_CORRECTION  (1964.0f / 1952.0f)
static const float GEAR_RATIOS[] = { 15.26f, 8.25f, 5.65f, 4.43f, 3.54f };
#define NUM_GEARS  5

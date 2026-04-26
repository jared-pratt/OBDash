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

// ── Set PID_TEST 1 to probe which PIDs your car supports at boot ─────
// Results printed to Serial. Set back to 0 after testing.
#define PID_TEST    0

// ── 2006 Toyota Corolla CE — C60 MT gear ratios ──────────────────────
// Speed correction: OBD speed under-reads with larger tires.
// Formula to adjust: new = old * (GPS_speed / displayed_speed)
// Example: GPS=65, display=63 → new = 1.025 * (65/63) = 1.058
#define SPEED_CORRECTION  1.016f

// Wheel circumference for 215/45/R17 (meters) — used in RPM-based gear detection
// 215*0.45*2 + 17*25.4 = 625.3mm diameter → π*0.6253 = 1.9639m
#define WHEEL_CIRC_M  1.9639f

// Starting odometer value — CarCoach adds miles on top of this
#define STARTING_ODOMETER  329741.0f

// Overall gear ratios (transmission × final drive 4.312) for C60 5-speed
// 1st:3.545×4.312=15.27 2nd:1.904×4.312=8.21 3rd:1.310×4.312=5.65
// 4th:1.028×4.312=4.43  5th:0.820×4.312=3.54
static const float GEAR_RATIOS[] = { 15.26f, 8.25f, 5.65f, 4.43f, 3.54f };
#define NUM_GEARS  5

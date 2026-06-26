#pragma once

// =============================================================================
// TANK NODE HARDWARE CONFIGURATION
// Pin definitions, NVS keys, and NodeConfig factory defaults.
// RF settings and network constants live in protocol.hpp (shared with all nodes).
// =============================================================================

// ── LoRa SPI + control pins (SX1262 E22 module) ──────────────────────────────
#define LORA_NSS   5
#ifdef CONFIG_IDF_TARGET_ESP32S3
  #define LORA_DIO1  16
  #define LORA_BUSY  17
  #define LORA_MOSI  11
  #define LORA_MISO  13
#else
  #define LORA_DIO1  26
  #define LORA_BUSY  27
  #define LORA_MOSI  23
  #define LORA_MISO  19
#endif
#define LORA_NRST  14
#define LORA_SCK   18

// TX power is node-specific (tank uses 22 dBm, TTGO uses 14 dBm)
#define LORA_TX_POWER    22

// ── WS2812B LED strip ─────────────────────────────────────────────────────────
#define WS2812B_PIN  48
#define NUM_LEDS     10

// ── I2C bus for PCF8574 I/O expander ─────────────────────────────────────────
//   P0–P2 : vertical float switches (mechanical reed, active-low — closed = LOW)
//   P3–P7 : TTP223 capacitive touch modules
// All 8 pins are written 0xFF (inputs with internal quasi-bidirectional pull-ups).
#define I2C_SDA  1
#define I2C_SCL  2

#define PCF8574_ADDR  0x27

// Float switch inputs (P0–P2): switch closed (water present) pulls pin LOW
#define PCF_FLOAT_0   0  // lowest float switch  (level ≥ 1)
#define PCF_FLOAT_1   1  // middle float switch   (level ≥ 2)
#define PCF_FLOAT_2   2  // upper float switch    (level = 3 / full)

// TTP223 capacitive touch module inputs (P3–P7)
// TTP223 default wiring (A-pad open, B-pad open):
//   idle (not touched) → output LOW  → PCF pin LOW
//   touched            → output HIGH → PCF pin HIGH
// We detect the rising edge (LOW→HIGH) as a touch event.
#define TTP223_TOUCHED_LEVEL  1   // 1 = active-HIGH (default), 0 = active-LOW (A-pad bridged)

#define PCF_TOUCH_MODE  3  // MODE toggle button
#define PCF_TOUCH_P1    4  // Pump 1 manual toggle
#define PCF_TOUCH_P2    5  // Pump 2 manual toggle
#define PCF_TOUCH_P3    6  // Pump 3 manual toggle
#define PCF_TOUCH_POND  7  // Pond pump manual toggle

// Legacy aliases — kept so existing code compiles without renaming every reference
#define PCF_BTN_MODE  PCF_TOUCH_MODE
#define PCF_BTN_P1    PCF_TOUCH_P1
#define PCF_BTN_P2    PCF_TOUCH_P2
#define PCF_BTN_P3    PCF_TOUCH_P3
#define PCF_BTN_POND  PCF_TOUCH_POND

// ── Relay and buzzer outputs (active-low: LOW=ON, HIGH=OFF) ──────────────────
#define RELAY_P1    47
#define RELAY_P2    41
#define RELAY_P3    39
#define BUZZER_PIN  21

// ── LED flash durations ───────────────────────────────────────────────────────
#define FLASH_RX_MS  200UL
#define FLASH_TX_MS  200UL

// ── NVS ──────────────────────────────────────────────────────────────────────
#define NVS_NAMESPACE    "tanknode"
#define NVS_CFG_VER_KEY  "cfgver"
#define NVS_CFG_KEY      "cfg"
#define CONFIG_VERSION   4  // bumped: overcurrent/dryrun fields removed

// ── NodeConfig factory defaults (applied on first boot or version mismatch) ──
#define DEF_PUMP_MIN_RUNTIME_MS        30000UL
#define DEF_PUMP_MIN_COOLDOWN_MS       60000UL
#define DEF_REPLENISH_RUNON_MS        300000UL
#define DEF_TELEMETRY_INTERVAL_MS      30000UL
#define DEF_NETWORK_TIMEOUT_MS         60000UL
#define DEF_ACK_TIMEOUT_MS            10000UL
#define DEF_ACK_MAX_RETRIES                5
#define DEF_BOOT_AUTO_MODE                 1

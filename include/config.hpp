#pragma once

// =============================================================================
// PIN DEFINITIONS
// =============================================================================

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

#define WS2812B_PIN  48
#define NUM_LEDS     10

// I2C bus for PCF8574 I/O expander
//   P0–P2 : vertical float switches (mechanical reed, active-low — closed = LOW)
//   P3–P7 : TTP223 capacitive touch modules
// All 8 pins are written 0xFF (inputs with internal quasi-bidirectional pull-ups).
#define I2C_SDA  1
#define I2C_SCL  2

// PCF8574 I/O expander — address and pin assignments
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
// If your TTP223 modules have the A-pad bridged (active-LOW mode) flip
// TTP223_TOUCHED_LEVEL to 0 and update the edge detection in task_input_sensor.hpp.
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

// Relay and buzzer outputs (native GPIO, active-low: LOW=ON, HIGH=OFF)
#define RELAY_P1    47
#define RELAY_P2    41
#define RELAY_P3    39
#define BUZZER_PIN  21

#define CURRENT_P1_ADC  4   // ADC1_CH3 on S3 / ADC2_CH0 on ESP32
// GPIO 39 is not ADC-capable on ESP32-S3; ADC1 is GPIO 1-10, ADC2 is GPIO 11-20
#ifdef CONFIG_IDF_TARGET_ESP32S3
  #define CURRENT_P2_ADC   7  // ADC1_CH6 on S3
#else
  #define CURRENT_P2_ADC  39  // ADC1_CH3 on classic ESP32 (input-only, safe)
#endif
#ifdef CONFIG_IDF_TARGET_ESP32S3
  #define CURRENT_P3_ADC  3  // ADC1_CH2 on S3 (GPIO 2 reserved for I2C_SCL)
#else
  #define CURRENT_P3_ADC  2  // ADC2_CH2 on classic ESP32
#endif

// =============================================================================
// COMPILE-TIME CONSTANTS AND DEFAULTS
// Items marked DEF_* seed NVS on first boot; after that gConfig is authoritative.
// =============================================================================

// LoRa RF  (fixed – not runtime-configurable)
#define LORA_FREQUENCY   868.0f
#define LORA_BANDWIDTH   125.0f
#define LORA_SF          9
#define LORA_CR          5
#define LORA_SYNC_WORD   0x12
#define LORA_TX_POWER    22
#define LORA_PREAMBLE    8

// Network identifiers
#define MY_NETWORK_MAGIC  0x5A6B
#define NODE_GATEWAY      0x01
#define NODE_TANK_LOCAL   0x02
#define NODE_POND_REMOTE  0x03
#define NODE_BROADCAST    0xFF

// Fixed operational constants (not user-tunable)
#define ADC_SAMPLES           8
#define FLASH_RX_MS           200UL
#define FLASH_TX_MS           200UL
#define FLASH_NET_MS          400UL

// NVS
#define NVS_NAMESPACE    "tanknode"
#define NVS_CFG_VER_KEY  "cfgver"
#define NVS_CFG_KEY      "cfg"
#define CONFIG_VERSION   2  // bumped: overcurrent_grace_ticks + fault_lockout_enabled added

// Consecutive abnormal-reset counter stored in NVS.
// If radio init crashes (INT_WDT/Panic) repeatedly we skip init to break the loop.
#define NVS_RADIO_STREAK_KEY  "radio_cs"
#define RADIO_SKIP_STREAK     3   // skip radio after this many consecutive crash-boots during init

// Defaults for NodeConfig (applied on first boot or after version mismatch)
#define DEF_PUMP_MIN_RUNTIME_MS        30000UL
#define DEF_PUMP_MIN_COOLDOWN_MS       60000UL
#define DEF_REPLENISH_RUNON_MS        300000UL
#define DEF_TELEMETRY_INTERVAL_MS      30000UL
#define DEF_NETWORK_TIMEOUT_MS         60000UL
#define DEF_ACK_TIMEOUT_MS            10000UL  // LoRa round-trip at 868/SF9 + margin
#define DEF_ACK_MAX_RETRIES                5
#define DEF_OVERCURRENT_THRESH          3200
#define DEF_DRYRUN_THRESH                150
#define DEF_BOOT_AUTO_MODE                 1
#define DEF_OVERCURRENT_GRACE_TICKS        5   // 5 × 50 ms = 250 ms inrush window
#define DEF_FAULT_LOCKOUT_ENABLED          1   // 1=kill relays on fault  0=warn only

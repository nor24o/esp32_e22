#pragma once

// =============================================================================
// TANK CONTROLLER — HARDWARE PIN DEFINITIONS AND DEFAULT CONFIG VALUES
//
// This file describes:
//   1. Which GPIO pins connect to which hardware on the tank controller board.
//   2. The default values for the runtime configuration (NodeConfig).
//
// RF settings (frequency, bandwidth, etc.) live in protocol.hpp because they
// are shared by all devices.  Only the TX power is here because it depends on
// the specific radio module used (tank uses a higher-power SX1262).
// =============================================================================

// ── LoRa module (SX1262 E22) pin connections ─────────────────────────────────
// Conditional compilation: ESP32-S3 uses different SPI pin numbers than classic ESP32.
#define LORA_NSS   5   // SPI chip-select (active low)
#ifdef CONFIG_IDF_TARGET_ESP32S3
  #define LORA_DIO1  16  // Interrupt line from SX1262 (TX done, RX done)
  #define LORA_BUSY  17  // Busy pin — wait for LOW before sending a command to SX1262
  #define LORA_MOSI  11
  #define LORA_MISO  13
#else
  #define LORA_DIO1  26
  #define LORA_BUSY  27
  #define LORA_MOSI  23
  #define LORA_MISO  19
#endif
#define LORA_NRST  14   // Hardware reset pin for SX1262
#define LORA_SCK   18   // SPI clock

// Tank uses 22 dBm; the test TTGO board uses 14 dBm (its SX1276 maximum).
#define LORA_TX_POWER    22

// ── WS2812B LED strip ─────────────────────────────────────────────────────────
#define WS2812B_PIN  48   // Data pin for the LED strip
#define NUM_LEDS     10   // Number of LEDs in the strip

// ── I2C bus for PCF8574 I/O expander ─────────────────────────────────────────
// The PCF8574 gives us 8 extra digital I/O pins over I2C.
//   P0–P2 : float switches (water level sensors, active-low: LOW when water is present)
//   P3–P7 : TTP223 capacitive touch buttons
#define I2C_SDA  1
#define I2C_SCL  2
#define PCF8574_ADDR  0x27   // I2C address of the expander chip

// PCF8574 bit positions for float switches (water level sensors)
#define PCF_FLOAT_0   0  // Lowest float switch  → water level ≥ 1
#define PCF_FLOAT_1   1  // Middle float switch  → water level ≥ 2
#define PCF_FLOAT_2   2  // Upper float switch   → water level = 3 (full)

// TTP223 touch button wiring note:
//   Default (A-pad open, B-pad open): idle = LOW, touched = HIGH
//   Set TTP223_TOUCHED_LEVEL to 0 if you bridge the A-pad (inverts polarity).
#define TTP223_TOUCHED_LEVEL  1   // 1 = active-HIGH (default TTP223 wiring)

// PCF8574 bit positions for touch buttons
#define PCF_TOUCH_MODE  3  // Toggle between AUTO and MANUAL mode
#define PCF_TOUCH_P1    4  // Toggle pump P1 (manual mode only)
#define PCF_TOUCH_P2    5  // Toggle pump P2 (manual mode only)
#define PCF_TOUCH_P3    6  // Toggle pump P3 (manual mode only)
#define PCF_TOUCH_POND  7  // Toggle pond pump (manual mode only)

// Aliases so older code that uses PCF_BTN_xxx still compiles
#define PCF_BTN_MODE  PCF_TOUCH_MODE
#define PCF_BTN_P1    PCF_TOUCH_P1
#define PCF_BTN_P2    PCF_TOUCH_P2
#define PCF_BTN_P3    PCF_TOUCH_P3
#define PCF_BTN_POND  PCF_TOUCH_POND

// ── Relay outputs ─────────────────────────────────────────────────────────────
// Relays are active-low: writing LOW to the GPIO energises the relay (pump ON).
#define RELAY_P1    47
#define RELAY_P2    41
#define RELAY_P3    39
#define BUZZER_PIN  21   // Also active-low

// ── LED flash durations ───────────────────────────────────────────────────────
#define FLASH_RX_MS  200UL   // LED stays lit for 200 ms on packet receive
#define FLASH_TX_MS  200UL   // LED stays lit for 200 ms on packet transmit

// ── NVS (Non-Volatile Storage) keys ──────────────────────────────────────────
// These keys identify the config and stats data stored in flash memory.
#define NVS_NAMESPACE    "tanknode"
#define NVS_CFG_VER_KEY  "cfgver"
#define NVS_CFG_KEY      "cfg"
// Bumping CONFIG_VERSION causes the device to ignore the old stored config and
// reset to the defaults below.  Do this whenever NodeConfig changes shape.
#define CONFIG_VERSION   5

// ── NodeConfig factory defaults ───────────────────────────────────────────────
// Applied on first boot or when CONFIG_VERSION changes.
#define DEF_PUMP_MIN_RUNTIME_MS        30000UL  // Pump runs at least 30 s before it can stop
#define DEF_PUMP_MIN_COOLDOWN_MS       60000UL  // Wait 60 s after stop before restarting
#define DEF_REPLENISH_RUNON_MS        300000UL  // Pond pump runs 5 min per tank refill
#define DEF_TELEMETRY_INTERVAL_MS      10000UL  // Send telemetry every 10 s
#define DEF_NETWORK_TIMEOUT_MS         60000UL  // Raise error if no peer message for 60 s
#define DEF_CMD_RESPONSE_TIMEOUT_MS    15000UL  // Wait 15 s for pond telemetry after a command
#define DEF_BOOT_AUTO_MODE                  1   // Start in auto mode after reboot

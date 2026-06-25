// =============================================================================
// TANK CONTROLLER NODE (0x02)  –  Production Firmware v1.1
// Target  : Espressif ESP32 (Xtensa LX6 Dual-Core)
// Framework: Arduino + FreeRTOS
// Libraries: RadioLib (SX1262), FastLED (WS2812B), Wire/PCF8574, Preferences
//
// Protocol v1.1 vs v1.0:
//   • TelemetryData gains last_rssi (int8) and last_snr (int8) — wire-breaking change
//   • MSG_CONFIG_GET/SET/RESP  (types 4/5/6) — runtime config over LoRa
//   • MSG_STATS_GET/RESP       (types 7/8)   — pump run-times, fill cycles, boot info
//   • All timing/threshold constants now live in NVS-backed NodeConfig (gConfig)
//
// HARDWARE NOTES:
//   • GPIO 34-39: input-only, no internal pull-up.  Install 10 kΩ to 3.3 V.
//   • GPIO 2  (I2C_SCL): used for PCF8574. CURRENT_P3_ADC moved to GPIO 3 on S3.
//   • Relays P1/P2/P3: GPIO 47/41/39 (active-low). Buzzer: GPIO 21 (active-low).
//   • WS2812B: GPIO 40. LoRa MISO: GPIO 13 (S3), 19 (ESP32). BUSY: GPIO 17 (S3), 27 (ESP32).
//   • RadioLib transmit() blocks; at SF9/125 kHz max payload ≈ 330 ms.
//   • DHT22 removed — temperature/humidity fields in telemetry will be zero.
//
// SOURCE LAYOUT:
//   include/config.hpp              — pin definitions, compile-time constants, defaults
//   include/protocol.hpp            — packed network structs (LoRaHeader, NodeConfig, …)
//   include/globals.hpp             — SystemState, FreeRTOS handles, peripheral instances
//   include/nvs_manager.hpp         — NVS config/stats load, save, validate
//   include/helpers.hpp             — packet builder / sender helpers
//   include/pump_hysteresis.hpp     — PumpHysteresis struct + timing helpers
//   include/task_input_sensor.hpp   — Task_InputSensorPoll
//   include/task_lora_transceiver.hpp — DIO1 ISR + Task_LoRaTransceiver
//   include/task_control_engine.hpp — Task_ControlEngine (main logic)
//   include/task_ui_animation.hpp   — Task_UIAnimation (WS2812B)
// =============================================================================

#include "config.hpp"
#include "protocol.hpp"
#include "globals.hpp"
#include "nvs_manager.hpp"
#include "helpers.hpp"
#include "pump_hysteresis.hpp"
#include "task_input_sensor.hpp"
#include "task_lora_transceiver.hpp"
#include "task_control_engine.hpp"
#include "task_ui_animation.hpp"

#include <esp_system.h>

// =============================================================================
// SETUP
// =============================================================================

static const char* resetReasonStr(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "Power-on";
        case ESP_RST_EXT:       return "External reset pin";
        case ESP_RST_SW:        return "Software reset";
        case ESP_RST_PANIC:     return "Panic / exception";
        case ESP_RST_INT_WDT:   return "Interrupt watchdog";
        case ESP_RST_TASK_WDT:  return "Task watchdog  ← likely a hang during init";
        case ESP_RST_WDT:       return "Other watchdog";
        case ESP_RST_BROWNOUT:  return "Brownout (low supply voltage)";
        case ESP_RST_DEEPSLEEP: return "Deep-sleep wakeup";
        default:                return "Unknown";
    }
}

void setup() {
    Serial.begin(115200);
    delay(3000);  // allow time for serial monitor to connect

    esp_reset_reason_t rr = esp_reset_reason();
    Serial.println("\n============================================================");
    Serial.println("[BOOT] Tank Controller Node 0x02 v1.1");
    Serial.printf( "[BOOT] Reset reason : %s\n", resetReasonStr(rr));
    if (rr == ESP_RST_TASK_WDT || rr == ESP_RST_INT_WDT || rr == ESP_RST_PANIC) {
        Serial.println("[BOOT] WARNING       : Abnormal reset — probably a hang or crash");
        Serial.println("[BOOT]                 during the previous boot's hardware init.");
        Serial.println("[BOOT]                 If this repeats, check SX1262 wiring.");
    }
    Serial.println("------------------------------------------------------------");

    // ── Load NVS config and stats ─────────────────────────────────────────────
    Serial.println("[INIT] Loading NVS config and stats...");
    loadConfig();
    gStats.boot_count++;
    {
        Preferences p;
        p.begin(NVS_NAMESPACE, false);
        p.putUShort("bc", gStats.boot_count);
        p.end();
    }
    Serial.printf("[BOOT] Boot #%u  |  auto=%u  |  fill_cycles=%u  |  last_fault=%u\n",
                  gStats.boot_count, gConfig.boot_auto_mode,
                  gStats.fill_cycles, gStats.last_fault);
    Serial.printf("[BOOT] Runtime  : P1=%lus  P2=%lus  P3=%lus  Pond=%lus\n",
                  gStats.runtime_p1_s, gStats.runtime_p2_s,
                  gStats.runtime_p3_s, gStats.runtime_pond_s);
    Serial.printf("[BOOT] Config   : min_run=%lus  cooldown=%lus  replenish=%lus\n",
                  gConfig.pump_min_runtime_ms / 1000,
                  gConfig.pump_min_cooldown_ms / 1000,
                  gConfig.replenish_runon_ms / 1000);
    Serial.printf("[BOOT] Radio crash streak: %u  (skip after %u)\n",
                  gRadioCrashStreak, (uint8_t)RADIO_SKIP_STREAK);
    Serial.println("------------------------------------------------------------");

    // ── GPIO ─────────────────────────────────────────────────────────────────
    // Relay outputs (active-low: HIGH = relay OFF)
    pinMode(RELAY_P1, OUTPUT); digitalWrite(RELAY_P1, HIGH);
    pinMode(RELAY_P2, OUTPUT); digitalWrite(RELAY_P2, HIGH);
    pinMode(RELAY_P3, OUTPUT); digitalWrite(RELAY_P3, HIGH);
    // Buzzer output (active-low: HIGH = silent)
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, HIGH);
    Serial.printf("[INIT] Relays: P1=GPIO%d  P2=GPIO%d  P3=GPIO%d  Buzzer=GPIO%d\n",
                  RELAY_P1, RELAY_P2, RELAY_P3, BUZZER_PIN);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    Serial.printf("[INIT] ADC pins: currentP1=GPIO%d  currentP2=GPIO%d  currentP3=GPIO%d\n",
                  CURRENT_P1_ADC, CURRENT_P2_ADC, CURRENT_P3_ADC);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.beginTransmission(PCF8574_ADDR);
    Wire.write(0xFF);  // all 8 pins: inputs with pull-ups enabled
    gPcfOk = (Wire.endTransmission() == 0);
    Serial.printf("[INIT] PCF8574 I2C: SDA=GPIO%d  SCL=GPIO%d  addr=0x%02X  %s\n",
                  I2C_SDA, I2C_SCL, PCF8574_ADDR,
                  gPcfOk ? "OK — float switches on bits 0-2, buttons on bits 3-7"
                         : "not found — float switch and button inputs disabled");

    // ── FastLED ───────────────────────────────────────────────────────────────
    Serial.printf("[INIT] WS2812B: GPIO%d  %d LEDs\n", WS2812B_PIN, NUM_LEDS);
    FastLED.addLeds<WS2812B, WS2812B_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(80);
    fill_solid(leds, NUM_LEDS, CRGB::Black);

    // ── Global state ──────────────────────────────────────────────────────────
    memset(&gState, 0, sizeof(gState));
    gState.autoMode              = (gConfig.boot_auto_mode != 0);
    gState.lastGatewayContact_ms = millis();
    gState.lastPondContact_ms    = millis();

    // ── FreeRTOS primitives ───────────────────────────────────────────────────
    Serial.println("[INIT] Creating FreeRTOS primitives...");
    xStateMutex = xSemaphoreCreateMutex();
    configASSERT(xStateMutex != nullptr);

    xI2cMutex = xSemaphoreCreateMutex();
    configASSERT(xI2cMutex != nullptr);

    xLoRaIrqSemaphore = xSemaphoreCreateBinary();
    configASSERT(xLoRaIrqSemaphore != nullptr);

    xTxQueue = xQueueCreate(8, sizeof(LoRaPacket));
    configASSERT(xTxQueue != nullptr);

    xRxQueue = xQueueCreate(8, sizeof(LoRaPacket));
    configASSERT(xRxQueue != nullptr);

    // ── RadioLib SX1262 ───────────────────────────────────────────────────────
    Serial.println("------------------------------------------------------------");
    Serial.printf("[INIT] SX1262 SPI : SCK=GPIO%d  MISO=GPIO%d  MOSI=GPIO%d  NSS=GPIO%d\n",
                  LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    Serial.printf("[INIT] SX1262 ctrl: RST=GPIO%d  BUSY=GPIO%d  DIO1=GPIO%d\n",
                  LORA_NRST, LORA_BUSY, LORA_DIO1);
    Serial.printf("[INIT] RF settings: %.1f MHz  BW=%.0f kHz  SF%d  CR4/%d  PWR=%d dBm\n",
                  LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SF, LORA_CR, LORA_TX_POWER);
    Serial.flush();  // ensure RF settings reach the monitor before any potentially-hanging operation

    // Crash-streak guard: if the last RADIO_SKIP_STREAK boots ended in INT_WDT or
    // Panic (likely during radio init), skip radio init entirely this boot and let
    // the user recover via standalone mode.  Cleared on a successful radio boot.
    bool skipRadioInit = false;
    if (gRadioCrashStreak >= RADIO_SKIP_STREAK) {
        skipRadioInit = true;
        Serial.printf("[WARN] Radio init skipped — %u consecutive crash-boots detected.\n", gRadioCrashStreak);
        Serial.println("[WARN] Booting in forced standalone mode.  Reset the crash counter by");
        Serial.println("[WARN] erasing NVS (pio run -t erase) or repairing the SX1262 wiring.");
    }

    // Increment streak now; reset to 0 after init completes (success or soft-failure).
    // A crash (INT_WDT/Panic) before the reset leaves the counter elevated in NVS.
    if (!skipRadioInit) {
        Preferences p;
        p.begin(NVS_NAMESPACE, false);
        p.putUChar(NVS_RADIO_STREAK_KEY, (uint8_t)(gRadioCrashStreak + 1));
        p.end();
    }

    if (!skipRadioInit) {
        // Pre-check BUSY with a pull-up so a floating (absent/unpowered) module reads HIGH.
        // The SX1262 actively drives BUSY LOW when idle, overriding the pull-up.
        // Without the pull-up a floating pin can read LOW and enter a crashing init path.
        pinMode(LORA_BUSY, INPUT_PULLUP);
        delayMicroseconds(200);  // let pull-up settle
        bool busyLow = (digitalRead(LORA_BUSY) == LOW);
        pinMode(LORA_BUSY, INPUT);  // release pull-up; RadioLib reconfigures during begin()

        if (!busyLow) {
            Serial.println("[WARN] LORA_BUSY not driven LOW → SX1262 absent, unpowered, or wiring error.");
            Serial.println("[WARN] Skipping radio init.  Check: module power, RST/BUSY/SPI wiring.");
            Serial.flush();
        } else {
            Serial.println("[INIT] LORA_BUSY is LOW → SX1262 detected. Initialising SPI...");
            Serial.flush();

            loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

            Serial.println("[INIT] SPI ready.  Calling radio.begin()...");
            Serial.flush();

            int radioState = radio.begin(
                LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SF,
                LORA_CR, LORA_SYNC_WORD, LORA_TX_POWER, LORA_PREAMBLE);

            if (radioState != RADIOLIB_ERR_NONE) {
                Serial.printf("[ERROR] radio.begin() failed — RadioLib code %d\n", radioState);
                Serial.println("[ERROR] Possible causes:");
                Serial.println("[ERROR]   • SPI wiring error  (SCK / MISO / MOSI / NSS)");
                Serial.println("[ERROR]   • RST or BUSY pin disconnected or shorted");
                Serial.println("[ERROR]   • Module not powered or damaged");
            } else {
                radio.setCRC(true);
                radio.setPacketReceivedAction(onPacketReceived);
                radioState = radio.startReceive();
                if (radioState != RADIOLIB_ERR_NONE) {
                    Serial.printf("[ERROR] radio.startReceive() failed — RadioLib code %d\n", radioState);
                } else {
                    gRadioOk = true;
                    Serial.printf("[BOOT] Radio OK  |  %.1f MHz  |  Packet size: %u bytes\n",
                                  LORA_FREQUENCY, (unsigned)sizeof(LoRaPacket));
                }
            }
        }
        // We reached this point without crashing — reset streak regardless of success/failure.
        // The streak only stays elevated if the device crashes (INT_WDT/Panic) before reaching here.
        {
            Preferences p;
            p.begin(NVS_NAMESPACE, false);
            p.putUChar(NVS_RADIO_STREAK_KEY, 0);
            p.end();
        }
    }

    if (!gRadioOk) {
        Serial.println("[WARN] *** STANDALONE MODE — LoRa disabled ***");
        Serial.println("[WARN]     Local pumps P1/P2/P3, float switches, buttons, LEDs: fully operational.");
        Serial.println("[WARN]     Pond pump and gateway communication: disabled.");
        Serial.println("[WARN]     Fix hardware and reset to enable LoRa.");
        Serial.flush();
        // Note: no FastLED.show() here — FastLED's legacy RMT backend on ESP32-S3
        // disables CPU interrupts during the busy-wait and can exceed the 300 ms
        // INT_WDT threshold.  UIAnimation (Core 1, 30 ms tick) will light the LEDs
        // as soon as the task scheduler starts.
    }
    Serial.println("------------------------------------------------------------");

    // ── FreeRTOS tasks ────────────────────────────────────────────────────────
    Serial.println("[INIT] Starting FreeRTOS tasks..."); Serial.flush();
    BaseType_t rc;

    // Core 1: InputSensorPoll P3, ControlEngine P4, UIAnimation P1
    //   FastLED RMT driver is initialised in setup() which runs on Core 1.
    //   Calling FastLED.show() from Core 0 can block the RMT wait with interrupts
    //   disabled long enough to trigger INT_WDT — keep UIAnimation on Core 1.
    // Core 0: LoRaTransceiver P2 (event-driven, no FastLED)
    rc = xTaskCreatePinnedToCore(Task_InputSensorPoll, "InputPoll", 4096, nullptr, 3, nullptr, 1);
    configASSERT(rc == pdPASS);
    Serial.println("[INIT]   InputSensorPoll  → Core 1  Priority 3  OK"); Serial.flush();

    rc = xTaskCreatePinnedToCore(Task_ControlEngine,   "CtrlEng",  8192, nullptr, 4, nullptr, 1);
    configASSERT(rc == pdPASS);
    Serial.println("[INIT]   ControlEngine    → Core 1  Priority 4  OK"); Serial.flush();

    rc = xTaskCreatePinnedToCore(Task_LoRaTransceiver, "LoRaTx",   8192, nullptr, 2, nullptr, 0);
    configASSERT(rc == pdPASS);
    Serial.println("[INIT]   LoRaTransceiver  → Core 0  Priority 2  OK"); Serial.flush();

    rc = xTaskCreatePinnedToCore(Task_UIAnimation,     "UIAnim",   8192, nullptr, 1, nullptr, 1);
    configASSERT(rc == pdPASS);
    Serial.println("[INIT]   UIAnimation      → Core 1  Priority 1  OK"); Serial.flush();

    Serial.println("============================================================");
    Serial.println("[BOOT] System operational.");
    Serial.println("============================================================");
}

// =============================================================================
// LOOP (all work is in RTOS tasks)
// =============================================================================

void loop() { vTaskDelay(portMAX_DELAY); }

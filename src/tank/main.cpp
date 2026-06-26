// =============================================================================
// TANK CONTROLLER NODE (0x02)  —  Production Firmware v1.1
// Target  : Espressif ESP32-S3 (or classic ESP32)
// Hardware: SX1262 E22 LoRa, WS2812B LED strip, PCF8574 I/O expander
//
// See include/ for source layout:
//   shared   protocol.hpp           — packet structs, node IDs, RF settings
//   tank     config.hpp             — pin definitions, NVS keys, DEF_* defaults
//            globals.hpp            — SystemState, FreeRTOS handles, peripherals
//            nvs_manager.hpp        — NVS config/stats load/save/validate
//            helpers.hpp            — packet builder / sender helpers
//            pump_hysteresis.hpp    — PumpHysteresis struct + timing helpers
//            task_input_sensor.hpp  — Task_InputSensorPoll (20 ms, Core 1 P3)
//            task_lora_transceiver.hpp — DIO1 ISR + Task_LoRaTransceiver (Core 0 P2)
//            task_control_engine.hpp   — Task_ControlEngine (50 ms, Core 1 P4)
//            task_ui_animation.hpp  — Task_UIAnimation (30 ms, Core 1 P1)
//            task_serial_console.hpp   — Task_SerialConsole (Core 0 P1)
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <FastLED.h>
#include <Wire.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_system.h>

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
#include "task_serial_console.hpp"

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
    delay(3000);

    esp_reset_reason_t rr = esp_reset_reason();
    Serial.println("\n============================================================");
    Serial.println("[BOOT] Tank Controller Node 0x02 v1.1");
    Serial.printf( "[BOOT] Reset reason : %s\n", resetReasonStr(rr));
    if (rr == ESP_RST_TASK_WDT || rr == ESP_RST_INT_WDT || rr == ESP_RST_PANIC) {
        Serial.println("[BOOT] WARNING       : Abnormal reset — check SX1262 wiring.");
    }
    Serial.println("------------------------------------------------------------");

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
    Serial.printf("[BOOT] Config   : min_run=%lus  cooldown=%lus  replenish=%lus  telemetry=%lus  cmd_timeout=%lus\n",
                  gConfig.pump_min_runtime_ms / 1000,
                  gConfig.pump_min_cooldown_ms / 1000,
                  gConfig.replenish_runon_ms / 1000,
                  gConfig.telemetry_interval_ms / 1000,
                  gConfig.cmd_response_timeout_ms / 1000);
    Serial.println("------------------------------------------------------------");

    // ── GPIO ─────────────────────────────────────────────────────────────────
    pinMode(RELAY_P1,   OUTPUT); digitalWrite(RELAY_P1,   HIGH);
    pinMode(RELAY_P2,   OUTPUT); digitalWrite(RELAY_P2,   HIGH);
    pinMode(RELAY_P3,   OUTPUT); digitalWrite(RELAY_P3,   HIGH);
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, HIGH);
    Serial.printf("[INIT] Relays: P1=GPIO%d  P2=GPIO%d  P3=GPIO%d  Buzzer=GPIO%d\n",
                  RELAY_P1, RELAY_P2, RELAY_P3, BUZZER_PIN);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.beginTransmission(PCF8574_ADDR);
    Wire.write(0xFF);
    gPcfOk = (Wire.endTransmission() == 0);
    Serial.printf("[INIT] PCF8574 I2C: SDA=GPIO%d  SCL=GPIO%d  addr=0x%02X  %s\n",
                  I2C_SDA, I2C_SCL, PCF8574_ADDR,
                  gPcfOk ? "OK — float switches P0-P2, TTP223 touch modules P3-P7"
                         : "not found — float switches and TTP223 touch inputs disabled");

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
    xStateMutex = xSemaphoreCreateMutex();         configASSERT(xStateMutex != nullptr);
    xI2cMutex   = xSemaphoreCreateMutex();         configASSERT(xI2cMutex   != nullptr);
    xLoRaIrqSemaphore = xSemaphoreCreateBinary();  configASSERT(xLoRaIrqSemaphore != nullptr);
    xTxQueue = xQueueCreate(8, sizeof(LoRaPacket)); configASSERT(xTxQueue != nullptr);
    xRxQueue = xQueueCreate(8, sizeof(LoRaPacket)); configASSERT(xRxQueue != nullptr);

    // ── RadioLib SX1262 ───────────────────────────────────────────────────────
    Serial.println("------------------------------------------------------------");
    Serial.printf("[INIT] SX1262 SPI : SCK=GPIO%d  MISO=GPIO%d  MOSI=GPIO%d  NSS=GPIO%d\n",
                  LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    Serial.printf("[INIT] SX1262 ctrl: RST=GPIO%d  BUSY=GPIO%d  DIO1=GPIO%d\n",
                  LORA_NRST, LORA_BUSY, LORA_DIO1);
    Serial.printf("[INIT] RF settings: %.1f MHz  BW=%.0f kHz  SF%d  CR4/%d  PWR=%d dBm\n",
                  LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SF, LORA_CR, LORA_TX_POWER);
    Serial.flush();

    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    // tcxoVoltage = 1.8 V: E22 powers its TCXO from DIO3 at 1.8 V.
    // Wrong voltage = radio appears to init OK but fails to receive.
    int radioState = radio.begin(
        LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SF,
        LORA_CR, LORA_SYNC_WORD, LORA_TX_POWER, LORA_PREAMBLE,
        1.8f);

    if (radioState != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] radio.begin() failed — RadioLib code %d\n", radioState);
        Serial.println("[ERROR]   -2 = chip not found (SPI/NSS/RST/BUSY wiring)");
        Serial.println("[ERROR]   -5 = calibration fail (check tcxoVoltage or wiring)");
    } else {
        Serial.println("[INIT] radio.begin() OK");
        radio.setDio2AsRfSwitch(true);
        radio.setCurrentLimit(140.0f);  // raise OCP for E22 external PA
        radio.setCRC(2);
        radio.setPacketReceivedAction(onPacketReceived);
        radioState = radio.startReceive();
        if (radioState != RADIOLIB_ERR_NONE) {
            Serial.printf("[ERROR] radio.startReceive() failed — code %d\n", radioState);
        } else {
            gRadioOk = true;
            Serial.printf("[BOOT] Radio OK  %.1f MHz  SF%d  BW%.0f  CR4/%d  SYNC=0x%02X  PWR=%d dBm\n",
                          LORA_FREQUENCY, LORA_SF, LORA_BANDWIDTH, LORA_CR,
                          LORA_SYNC_WORD, LORA_TX_POWER);
            Serial.printf("[BOOT]            %u-byte packets  preamble=%u symbols\n",
                          (unsigned)sizeof(LoRaPacket), LORA_PREAMBLE);
        }
    }

    if (!gRadioOk) {
        Serial.println("[WARN] *** STANDALONE MODE — LoRa disabled ***");
        Serial.println("[WARN]     Local pumps P1/P2/P3, float switches, buttons, LEDs: fully operational.");
        Serial.println("[WARN]     Pond pump and gateway communication: disabled.");
    }
    Serial.println("------------------------------------------------------------");

    // ── FreeRTOS tasks ────────────────────────────────────────────────────────
    Serial.println("[INIT] Starting FreeRTOS tasks..."); Serial.flush();
    BaseType_t rc;

    rc = xTaskCreatePinnedToCore(Task_InputSensorPoll, "InputPoll", 4096, nullptr, 3, nullptr, 1);
    configASSERT(rc == pdPASS);
    Serial.println("[INIT]   InputSensorPoll  → Core 1  Priority 3  OK"); Serial.flush();

    rc = xTaskCreatePinnedToCore(Task_ControlEngine, "CtrlEng", 8192, nullptr, 4, &xControlEngineTask, 1);
    configASSERT(rc == pdPASS);
    Serial.println("[INIT]   ControlEngine    → Core 1  Priority 4  OK"); Serial.flush();

    rc = xTaskCreatePinnedToCore(Task_LoRaTransceiver, "LoRaTx", 8192, nullptr, 2, nullptr, 0);
    configASSERT(rc == pdPASS);
    Serial.println("[INIT]   LoRaTransceiver  → Core 0  Priority 2  OK"); Serial.flush();

    rc = xTaskCreatePinnedToCore(Task_UIAnimation, "UIAnim", 8192, nullptr, 1, nullptr, 1);
    configASSERT(rc == pdPASS);
    Serial.println("[INIT]   UIAnimation      → Core 1  Priority 1  OK"); Serial.flush();

    rc = xTaskCreatePinnedToCore(Task_SerialConsole, "SerCon", 4096, nullptr, 1, nullptr, 0);
    configASSERT(rc == pdPASS);
    Serial.println("[INIT]   SerialConsole    → Core 0  Priority 1  OK"); Serial.flush();

    Serial.println("============================================================");
    Serial.println("[BOOT] System operational.");
    Serial.println("============================================================");
}

// =============================================================================
// LOOP (all work is in RTOS tasks)
// =============================================================================

void loop() { vTaskDelay(portMAX_DELAY); }

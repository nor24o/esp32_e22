#pragma once

#include "globals.hpp"
#include "helpers.hpp"

// =============================================================================
// TASK: INPUT SENSOR POLL  (Priority 3, Core 1, 20 ms)
// =============================================================================

static void Task_InputSensorPoll(void *pvParams) {
    const uint8_t curPins[3] = { CURRENT_P1_ADC, CURRENT_P2_ADC, CURRENT_P3_ADC };
    uint8_t  prevBits        = 0xFF;
    uint32_t pcfRetryAt_ms   = 0;  // millis() timestamp for next PCF re-init attempt

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));

        // ── PCF8574 hot-plug recovery (retry every 5 s while not present) ────
        if (!gPcfOk && (millis() >= pcfRetryAt_ms)) {
            pcfRetryAt_ms = millis() + 5000;
            if (xSemaphoreTake(xI2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                Wire.beginTransmission(PCF8574_ADDR);
                Wire.write(0xFF);  // all inputs with pull-ups
                bool ok = (Wire.endTransmission() == 0);
                xSemaphoreGive(xI2cMutex);
                if (ok) {
                    gPcfOk = true;
                    prevBits = 0xFF;
                    Serial.println("\n[PCF] PCF8574 detected — float switch and button inputs enabled");
                }
            }
        }

        // Read all 8 PCF8574 pins in one I2C transaction (skipped if not present)
        uint8_t currBits = 0xFF;  // safe default: all HIGH → no water, no button presses
        if (gPcfOk && xSemaphoreTake(xI2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            Wire.requestFrom((uint8_t)PCF8574_ADDR, (uint8_t)1);
            if (Wire.available() >= 1) {
                currBits = (uint8_t)Wire.read();
            }
            xSemaphoreGive(xI2cMutex);
        }

        // Falling-edge mask: bits that transitioned HIGH→LOW this tick (active-low press)
        uint8_t fell = prevBits & ~currBits;
        prevBits = currBits;

        // Water level: count consecutive LOW float bits from bit 0 upward
        uint8_t level = 0;
        for (uint8_t i = 0; i < 3; i++) {
            if ((currBits & (1u << i)) == 0) level = (uint8_t)(i + 1);
            else break;
        }

        // ADC current sensing (native analog pins, unchanged)
        uint16_t cur[3];
        for (uint8_t i = 0; i < 3; i++) cur[i] = readCurrentADC(curPins[i]);

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            gState.waterLevel = level;
            gState.currentP1  = cur[0];
            gState.currentP2  = cur[1];
            gState.currentP3  = cur[2];
            if (fell & (1u << PCF_BTN_MODE)) gState.btnMode_edge = true;
            if (fell & (1u << PCF_BTN_P1))   gState.btnP1_edge   = true;
            if (fell & (1u << PCF_BTN_P2))   gState.btnP2_edge   = true;
            if (fell & (1u << PCF_BTN_P3))   gState.btnP3_edge   = true;
            if (fell & (1u << PCF_BTN_POND))  gState.btnPond_edge  = true;
            xSemaphoreGive(xStateMutex);
        }
    }
}

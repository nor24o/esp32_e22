#pragma once

#include "globals.hpp"
#include "helpers.hpp"

// =============================================================================
// TASK: INPUT SENSOR POLL  (Priority 3, Core 1, 20 ms)
// =============================================================================

static void Task_InputSensorPoll(void *pvParams) {
    uint8_t  prevBits        = 0x07;  // P0-P2 float idle HIGH, P3-P7 TTP223 idle LOW
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
                    prevBits = 0x07;
                    Serial.println("\n[PCF] PCF8574 detected — float switches (P0-P2) and TTP223 touch modules (P3-P7) enabled");
                }
            }
        }

        // Read all 8 PCF8574 pins in one I2C transaction (skipped if not present).
        //   P0-P2 float switches idle HIGH (open), water present = LOW
        //   P3-P7 TTP223 default idle LOW, touched = HIGH  (TTP223_TOUCHED_LEVEL == 1)
        uint8_t currBits = 0x00;
        currBits |= 0x07;  // P0-P2 float switches: idle HIGH = no water
        if (gPcfOk && xSemaphoreTake(xI2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            Wire.requestFrom((uint8_t)PCF8574_ADDR, (uint8_t)1);
            if (Wire.available() >= 1) {
                currBits = (uint8_t)Wire.read();
            }
            xSemaphoreGive(xI2cMutex);
        }

        // Float switches (P0-P2): falling edge = water level rose (float closed, pin LOW)
        // TTP223 touch (P3-P7):   rising edge  = touched (TTP223 output went HIGH)
#if TTP223_TOUCHED_LEVEL == 1
        uint8_t touchEdge = ~prevBits & currBits & 0xF8;  // bits 3-7 only, LOW→HIGH
#else
        uint8_t touchEdge = prevBits & ~currBits & 0xF8;  // bits 3-7 only, HIGH→LOW
#endif
        prevBits = currBits;

        // Water level: count consecutive LOW float bits (P0-P2) from bit 0 upward
        uint8_t level = 0;
        for (uint8_t i = 0; i < 3; i++) {
            if ((currBits & (1u << i)) == 0) level = (uint8_t)(i + 1);
            else break;
        }

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            gState.waterLevel = level;
            if (touchEdge & (1u << PCF_TOUCH_MODE)) gState.btnMode_edge = true;
            if (touchEdge & (1u << PCF_TOUCH_P1))   gState.btnP1_edge   = true;
            if (touchEdge & (1u << PCF_TOUCH_P2))   gState.btnP2_edge   = true;
            if (touchEdge & (1u << PCF_TOUCH_P3))   gState.btnP3_edge   = true;
            if (touchEdge & (1u << PCF_TOUCH_POND))  gState.btnPond_edge  = true;
            xSemaphoreGive(xStateMutex);
        }
    }
}

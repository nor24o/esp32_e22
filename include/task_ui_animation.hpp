#pragma once

#include "globals.hpp"

// =============================================================================
// TASK: UI ANIMATION  (Priority 1, Core 1, 30 ms)
// =============================================================================

static void Task_UIAnimation(void *pvParams) {
    uint32_t lastRxFlash_ms = 0;
    uint32_t lastTxFlash_ms = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(30));

        uint32_t now = millis();

        uint8_t  waterLevel, errorCode;
        bool     autoMode, awaitingAck;
        bool     relay_p1, relay_p2, relay_p3, pondPump;
        bool     rxFlash, txFlash;

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) != pdTRUE) continue;

        waterLevel   = gState.waterLevel;
        autoMode     = gState.autoMode;
        awaitingAck  = gState.awaitingAck;
        errorCode    = gState.errorCode;
        relay_p1     = gState.relay_p1;
        relay_p2     = gState.relay_p2;
        relay_p3     = gState.relay_p3;
        pondPump     = gState.pondPump;
        rxFlash = gState.rxFlash; gState.rxFlash = false;
        txFlash = gState.txFlash; gState.txFlash = false;

        xSemaphoreGive(xStateMutex);

        if (rxFlash) lastRxFlash_ms = now;
        if (txFlash) lastTxFlash_ms = now;

        fill_solid(leds, NUM_LEDS, CRGB::Black);

        // --- LEDs 0-3: water level bar ---
        if (waterLevel == 0) {
            for (uint8_t i = 0; i < 4; i++) leds[i] = CRGB(40, 0, 0);  // empty: dim red
        } else {
            for (uint8_t i = 0; i < 4; i++) {
                if (i < waterLevel)        leds[i] = CRGB(0, 160, 80);  // filled: cyan-green
                else if (i == waterLevel)  leds[i] = CRGB(0, 15, 8);    // surface marker: faint
                else                       leds[i] = CRGB(0, 0, 6);     // empty upper: dark hint
            }
            if (waterLevel >= 3) leds[3] = CRGB(0, 80, 200);            // full: bright blue
        }

        // --- LED 4: system status ---
        if (errorCode == 3) {
            leds[4] = CRGB(200, 0, 0);           // no-comms: solid red
        } else if ((now - lastRxFlash_ms) < FLASH_RX_MS) {
            leds[4] = CRGB(148, 0, 211);         // RX: purple
        } else if ((now - lastTxFlash_ms) < FLASH_TX_MS) {
            leds[4] = CRGB(0, 220, 220);         // TX: cyan
        } else if (awaitingAck) {
            leds[4] = CRGB(180, 180, 0);         // awaiting ack: yellow
        } else if (autoMode) {
            leds[4] = CRGB(0, 180, 0);           // auto OK: green
        } else {
            leds[4] = CRGB(0, 0, 160);           // manual: blue
        }

        // --- LED 5: mode indicator ---
        leds[5] = autoMode ? CRGB(0, 120, 0) : CRGB(0, 0, 120);

        // --- LEDs 6-8: local relay states ---
        leds[6] = relay_p1 ? CRGB(0, 180, 0) : CRGB(25, 0, 0);
        leds[7] = relay_p2 ? CRGB(0, 180, 0) : CRGB(25, 0, 0);
        leds[8] = relay_p3 ? CRGB(0, 180, 0) : CRGB(25, 0, 0);

        // --- LED 9: remote pond pump ---
        if (!gRadioOk)     leds[9] = CRGB(30, 8, 0);     // no radio: dim orange
        else if (pondPump)  leds[9] = CRGB(0, 160, 160);  // running: cyan
        else                leds[9] = CRGB(0, 0, 30);     // idle: dim blue

        FastLED.show();
    }
}

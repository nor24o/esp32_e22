#pragma once

#include "globals.hpp"

// =============================================================================
// TASK: UI ANIMATION  (Priority 1, Core 0, 30 ms)
// =============================================================================

static void Task_UIAnimation(void *pvParams) {
    // slow ~1.9 s cycle (256 steps × 30 ms / 4)
    uint8_t  pulsePhase   = 0;
    // fast ~384 ms cycle (256 / 20 × 30 ms) — fault strobe, ACK pulse
    uint8_t  fastPhase    = 0;
    // water shimmer wave ~700 ms cycle (256 / 11 × 30 ms)
    uint8_t  shimmerPhase = 0;
    // pump breathing ~640 ms cycle (256 / 12 × 30 ms)
    uint8_t  pumpPhase    = 0;

    uint32_t lastRxFlash_ms = 0;
    uint32_t lastTxFlash_ms = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(30));

        uint32_t now = millis();

        uint8_t  waterLevel, errorCode;
        bool     autoMode, faultLockout, awaitingAck;
        bool     relay_p1, relay_p2, relay_p3, pondPump;
        bool     rxFlash, txFlash;

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) != pdTRUE) continue;

        waterLevel   = gState.waterLevel;
        autoMode     = gState.autoMode;
        faultLockout = gState.faultLockout;
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

        pulsePhase   += 4;
        fastPhase    += 20;
        shimmerPhase += 11;
        pumpPhase    += 12;

        fill_solid(leds, NUM_LEDS, CRGB::Black);

        // --- LEDs 0-3: water level bar (3 float switches → 4 LEDs) ---
        // level 0 = empty (all 4 pulse dim red)
        // level 1 = low   (LED 0 lit, LED 1 surface ripple, 2-3 dark hint)
        // level 2 = mid   (LEDs 0-1 lit, LED 2 surface ripple, LED 3 dark hint)
        // level 3 = full  (LEDs 0-2 lit, LED 3 bright pulsing full-indicator)
        if (waterLevel == 0) {
            uint8_t bri = scale8(sin8(pulsePhase), 50);
            for (uint8_t i = 0; i < 4; i++) leds[i] = CRGB(bri, 0, 0);
        } else {
            for (uint8_t i = 0; i < 4; i++) {
                if (i < waterLevel) {
                    // lit water body — sine shimmer wave across LEDs
                    uint8_t s = 150 + scale8(sin8(shimmerPhase + i * 64), 70);
                    leds[i] = CRGB(0, s, s >> 1);
                } else if (i == waterLevel) {
                    // surface LED — faint ripple one step above fill line
                    uint8_t surf = scale8(sin8((uint8_t)(shimmerPhase * 2 + 128)), 25);
                    leds[i] = CRGB(0, surf, surf);
                } else {
                    leds[i] = CRGB(0, 0, 6);  // empty upper zone: dark blue hint
                }
            }
            if (waterLevel >= 3) {
                // tank full: LED 3 pulses bright aqua as a "full" indicator
                uint8_t bri = 180 + scale8(sin8((uint8_t)(pulsePhase * 3)), 75);
                leds[3] = CRGB(0, bri >> 1, bri);
            }
        }

        // --- LED 4: system status ---
        if (faultLockout || errorCode == 1 || errorCode == 2) {
            // hard fault: fast red strobe
            leds[4] = (fastPhase < 128) ? CRGB(255, 0, 0) : CRGB(25, 0, 0);

        } else if (errorCode == 3) {
            // comms error: slow red breathing
            uint8_t bri = scale8(sin8(pulsePhase), 200) + 20;
            leds[4] = CRGB(bri, 0, 0);

        } else if ((now - lastRxFlash_ms) < FLASH_RX_MS) {
            leds[4] = CRGB(148, 0, 211);   // purple RX flash

        } else if ((now - lastTxFlash_ms) < FLASH_TX_MS) {
            leds[4] = CRGB(0, 220, 220);   // cyan TX flash

        } else if (awaitingAck) {
            // fast yellow triangle pulse
            leds[4] = CRGB(triwave8(fastPhase), triwave8(fastPhase), 0);

        } else if (autoMode) {
            // auto OK: slow breathing green
            uint8_t bri = scale8(sin8(pulsePhase), 180) + 40;
            leds[4] = CRGB(0, bri, 0);

        } else {
            leds[4] = CRGB(0, 0, 160);     // manual: steady blue
        }

        // --- LED 5: mode indicator ---
        if (autoMode) {
            uint8_t bri = scale8(sin8((uint8_t)(pulsePhase + 128)), 100) + 80;
            leds[5] = CRGB(0, bri, 0);     // breathing green
        } else {
            leds[5] = CRGB(0, 0, 120);     // steady blue
        }

        // --- LEDs 6-8: local relay states ---
        // running: breathing green (each pump offset 120° so they pulse independently)
        // off: dim dark red
        const bool relays[3] = { relay_p1, relay_p2, relay_p3 };
        for (uint8_t i = 0; i < 3; i++) {
            if (relays[i]) {
                uint8_t bri = scale8(sin8((uint8_t)(pumpPhase + i * 85)), 150) + 80;
                leds[6 + i] = CRGB(0, bri, 0);
            } else {
                leds[6 + i] = CRGB(25, 0, 0);
            }
        }

        // --- LED 9: remote pond pump ---
        if (!gRadioOk) {
            // no radio: dim orange slow pulse
            uint8_t bri = scale8(sin8(pulsePhase), 30) + 15;
            leds[9] = CRGB(bri, bri >> 2, 0);
        } else if (pondPump) {
            // running: breathing cyan (phase offset from local pumps)
            uint8_t bri = scale8(sin8((uint8_t)(pumpPhase + 170)), 160) + 60;
            leds[9] = CRGB(0, bri, bri);
        } else {
            leds[9] = CRGB(0, 0, 30);      // idle + radio OK: dim blue
        }

        FastLED.show();
    }
}

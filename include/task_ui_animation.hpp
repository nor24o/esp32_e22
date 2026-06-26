#pragma once

#include "globals.hpp"

// =============================================================================
// TASK: UI ANIMATION  (Priority 1, Core 1, runs every 30 ms)
//
// Drives the WS2812B LED strip.  Each LED shows a specific piece of system status.
//
// LED layout (10 LEDs total):
//
//   LEDs 0–3  Water level bar
//             0 = empty → all dim red
//             1 = one float switch active → first LED cyan-green
//             2 = two switches → two LEDs lit
//             3 = full → three LEDs cyan-green, LED 3 bright blue
//
//   LED 4     Radio / system status
//             Purple   → just received a packet
//             Cyan     → just transmitted a packet
//             Red      → no-comms error (lost contact with a peer)
//             Yellow   → waiting for pond telemetry to confirm a command
//             Green    → auto mode, all OK
//             Blue     → manual mode, all OK
//
//   LED 5     Operating mode
//             Green → auto mode
//             Blue  → manual mode
//
//   LEDs 6–8  Local relay states (P1, P2, P3)
//             Green → pump ON,  dim red → pump OFF
//
//   LED 9     Pond pump state
//             Cyan     → pond pump running
//             Dim blue → pond pump idle
//             Dim orange → radio not available (no LoRa)
// =============================================================================

static void Task_UIAnimation(void *pvParams) {
    uint32_t lastRxFlash_ms = 0;
    uint32_t lastTxFlash_ms = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(30));

        uint32_t now = millis();

        // Snapshot shared state.
        uint8_t  waterLevel, errorCode;
        bool     autoMode, pondCmdPending;
        bool     relay_p1, relay_p2, relay_p3, pondPump;
        bool     rxFlash, txFlash;

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) != pdTRUE) continue;

        waterLevel     = gState.waterLevel;
        autoMode       = gState.autoMode;
        pondCmdPending = gState.pondCmdPending;
        errorCode      = gState.errorCode;
        relay_p1       = gState.relay_p1;
        relay_p2       = gState.relay_p2;
        relay_p3       = gState.relay_p3;
        pondPump       = gState.pondPump;

        // Read flash flags and immediately clear them so each flash only shows once.
        rxFlash = gState.rxFlash; gState.rxFlash = false;
        txFlash = gState.txFlash; gState.txFlash = false;

        xSemaphoreGive(xStateMutex);

        if (rxFlash) lastRxFlash_ms = now;
        if (txFlash) lastTxFlash_ms = now;

        // Start with all LEDs off.
        fill_solid(leds, NUM_LEDS, CRGB::Black);

        // ── LEDs 0–3: water level bar ─────────────────────────────────────────
        if (waterLevel == 0) {
            // Tank completely empty — alert with dim red on all four LEDs.
            for (uint8_t i = 0; i < 4; i++) leds[i] = CRGB(40, 0, 0);
        } else {
            for (uint8_t i = 0; i < 4; i++) {
                if (i < waterLevel)       leds[i] = CRGB(0, 160, 80);   // water present
                else if (i == waterLevel) leds[i] = CRGB(0, 15,  8);    // surface marker
                else                      leds[i] = CRGB(0, 0,   6);    // empty above
            }
            if (waterLevel >= 3) leds[3] = CRGB(0, 80, 200);            // full: bright blue
        }

        // ── LED 4: radio / system status ─────────────────────────────────────
        if (errorCode == 3) {
            leds[4] = CRGB(200, 0, 0);           // no-comms error: solid red
        } else if ((now - lastRxFlash_ms) < FLASH_RX_MS) {
            leds[4] = CRGB(148, 0, 211);         // receiving: purple
        } else if ((now - lastTxFlash_ms) < FLASH_TX_MS) {
            leds[4] = CRGB(0, 220, 220);         // transmitting: cyan
        } else if (pondCmdPending) {
            leds[4] = CRGB(180, 180, 0);         // waiting for pond confirmation: yellow
        } else if (autoMode) {
            leds[4] = CRGB(0, 180, 0);           // auto mode, all good: green
        } else {
            leds[4] = CRGB(0, 0, 160);           // manual mode: blue
        }

        // ── LED 5: operating mode ─────────────────────────────────────────────
        leds[5] = autoMode ? CRGB(0, 120, 0) : CRGB(0, 0, 120);

        // ── LEDs 6–8: local pump relays ───────────────────────────────────────
        leds[6] = relay_p1 ? CRGB(0, 180, 0) : CRGB(25, 0, 0);
        leds[7] = relay_p2 ? CRGB(0, 180, 0) : CRGB(25, 0, 0);
        leds[8] = relay_p3 ? CRGB(0, 180, 0) : CRGB(25, 0, 0);

        // ── LED 9: pond pump state ────────────────────────────────────────────
        if (!gRadioOk) {
            leds[9] = CRGB(30, 8, 0);     // no radio: dim orange
        } else if (pondPump) {
            leds[9] = CRGB(0, 160, 160);  // pond pump running: cyan
        } else {
            leds[9] = CRGB(0, 0, 30);     // pond pump idle: dim blue
        }

        FastLED.show();
    }
}

#pragma once

#include "globals.hpp"

// =============================================================================
// ISR: DIO1 interrupt from the SX1262
//
// The SX1262 raises DIO1 when it finishes transmitting or receiving a packet.
// This ISR runs in interrupt context (very short — only gives a semaphore)
// and wakes the LoRaTransceiver task which does the actual reading/writing.
// IRAM_ATTR ensures the function lives in fast IRAM, required for ISRs.
// =============================================================================

void IRAM_ATTR onPacketReceived() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xLoRaIrqSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// =============================================================================
// TASK: LORA TRANSCEIVER  (Priority 2, Core 0, event-driven)
//
// This task owns the SX1262 radio.  Everything else in the system talks to the
// radio through this task — no other task touches the radio directly.
//
// TX side: other tasks put LoRaPacket structs into xTxQueue.
//          This task dequeues them and transmits one at a time.
//          After each TX the radio is put back into receive mode.
//
//          Before every transmission this task performs a CAD scan
//          (Channel Activity Detection).  CAD takes ~a few milliseconds:
//          the SX1262 listens for a LoRa preamble on the channel and
//          reports whether another node is already transmitting.
//          If the channel is busy the packet is re-queued and we back off
//          for a random 100–500 ms, preventing a collision that would
//          corrupt both packets.  If CAD itself fails for any reason we
//          transmit anyway (fail-open) so a single bad CAD result does
//          not permanently block the queue.
//
// RX side: the DIO1 ISR (above) signals xLoRaIrqSemaphore when a packet arrives.
//          This task reads the packet, validates the header, and puts it into
//          xRxQueue for the ControlEngine task to process.
//
// Why separate tasks?  Transmitting takes ~415 ms of air time.  Doing that in
// the ControlEngine would freeze all pump logic for nearly half a second.
// Running it on Core 0 lets Core 1 (ControlEngine, sensors, UI) keep going.
// =============================================================================

static void Task_LoRaTransceiver(void *pvParams) {

    if (!gRadioOk) {
        // Radio initialisation failed in setup() — silently drain the TX queue
        // so it does not fill up and block callers, then do nothing forever.
        LoRaPacket discard;
        for (;;) {
            while (xQueueReceive(xTxQueue, &discard, 0) == pdTRUE) {}
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    uint8_t rxBuf[sizeof(LoRaPacket)];

    for (;;) {

        // ── Transmit (non-blocking check) ────────────────────────────────────
        // If there is a packet waiting in the TX queue, send it now.
        LoRaPacket txPkt;
        if (xQueueReceive(xTxQueue, &txPkt, 0) == pdTRUE) {

            Serial.printf("\n[LoRa] TX type=%u → 0x%02X (%u bytes)... ",
                          txPkt.header.msg_type, txPkt.header.target_id,
                          (unsigned)sizeof(LoRaPacket));

            // ── CAD: listen before talk ───────────────────────────────────
            // scanChannel() makes the SX1262 perform a Channel Activity
            // Detection scan (~a few ms).  Returns RADIOLIB_LORA_DETECTED
            // if another node's preamble is on the air right now.
            int cadResult = radio.scanChannel();
            if (cadResult == RADIOLIB_LORA_DETECTED) {
                uint32_t backoffMs = 100 + (esp_random() % 400); // 100–500 ms
                Serial.printf("CAD: channel busy — backoff %lu ms\n", backoffMs);
                // Put the packet back at the front of the queue so it is
                // sent before any newer packets that arrive during the wait.
                xQueueSendToFront(xTxQueue, &txPkt, 0);
                radio.startReceive();          // keep listening while we wait
                vTaskDelay(pdMS_TO_TICKS(backoffMs));
                continue;                      // restart the loop iteration
            }
            if (cadResult != RADIOLIB_ERR_NONE) {
                // CAD itself failed (wiring glitch, etc.) — transmit anyway
                // rather than permanently blocking the queue.
                Serial.printf("CAD failed (code=%d) — transmitting anyway\n", cadResult);
            }

            // startTransmit() begins the transmission and returns immediately.
            // The CPU yields via xSemaphoreTake while the ~415 ms air time elapses.
            int state = radio.startTransmit((uint8_t *)&txPkt, (size_t)sizeof(LoRaPacket));
            if (state == RADIOLIB_ERR_NONE) {
                // Wait for the TX-done interrupt (1 s timeout is well above the air time).
                if (xSemaphoreTake(xLoRaIrqSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
                    state = radio.finishTransmit();
                } else {
                    state = -1;   // TX-done IRQ never fired — radio may be stuck
                }
            }

            if (state == RADIOLIB_ERR_NONE) {
                Serial.println("OK");
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    gState.txFlash        = true;
                    gState.txFlashTime_ms = millis();
                    xSemaphoreGive(xStateMutex);
                }
            } else {
                Serial.printf("FAILED (code=%d)\n", state);
            }

            // Put the radio back into receive mode after every transmission.
            int rxState = radio.startReceive();
            if (rxState != RADIOLIB_ERR_NONE) {
                Serial.printf("[LoRa] startReceive after TX failed: %d\n", rxState);
            }
        }

        // ── Receive (wait up to 10 ms for an interrupt) ───────────────────────
        if (xSemaphoreTake(xLoRaIrqSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            int rxLen = radio.getPacketLength();

            if (rxLen == (int)sizeof(LoRaPacket)) {
                int state = radio.readData(rxBuf, (size_t)sizeof(LoRaPacket));

                if (state == RADIOLIB_ERR_NONE) {
                    int8_t rssi = (int8_t)radio.getRSSI();
                    int8_t snr  = (int8_t)radio.getSNR();

                    LoRaPacket *pkt = reinterpret_cast<LoRaPacket *>(rxBuf);

                    // Reject packets from a different network or addressed to someone else.
                    bool isOurNetwork  = (pkt->header.magic_word == MY_NETWORK_MAGIC);
                    bool addressedToUs = (pkt->header.target_id == NODE_TANK_LOCAL ||
                                         pkt->header.target_id == NODE_BROADCAST);

                    if (!isOurNetwork || !addressedToUs) {
                        radio.startReceive();
                        continue;
                    }

                    // Pass to ControlEngine via the RX queue and wake it immediately.
                    if (xQueueSend(xRxQueue, pkt, 0) == pdTRUE) {
                        if (xControlEngineTask) xTaskNotifyGive(xControlEngineTask);
                        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                            gState.rxFlash        = true;
                            gState.rxFlashTime_ms = millis();
                            gState.lastRssi       = rssi;
                            gState.lastSnr        = snr;
                            xSemaphoreGive(xStateMutex);
                        }
                    }
                } else {
                    Serial.printf("[LoRa] RX read error: %d\n", state);
                }

            } else if (rxLen > 0) {
                // Wrong-length packet — discard it.
                radio.readData(rxBuf, (size_t)min(rxLen, (int)sizeof(rxBuf)));
            }

            radio.startReceive();
        }
    }
}

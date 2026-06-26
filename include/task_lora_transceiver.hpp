#pragma once

#include "globals.hpp"

// =============================================================================
// RADIOLIB DIO1 ISR
// =============================================================================

void IRAM_ATTR onPacketReceived() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xLoRaIrqSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// =============================================================================
// TASK: LORA TRANSCEIVER  (Priority 2, Core 0, Event-driven)
// =============================================================================

static void Task_LoRaTransceiver(void *pvParams) {
    if (!gRadioOk) {
        // Radio not initialised — silently drain TX queue and idle.
        LoRaPacket discard;
        for (;;) {
            while (xQueueReceive(xTxQueue, &discard, 0) == pdTRUE) {}
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    uint8_t rxBuf[sizeof(LoRaPacket)];

    for (;;) {
        // Drain TX queue first (non-blocking)
        LoRaPacket txPkt;
        if (xQueueReceive(xTxQueue, &txPkt, 0) == pdTRUE) {
            Serial.printf("\n[LoRa] TX type=%u target=0x%02X %uB ... ",
                          txPkt.header.msg_type, txPkt.header.target_id,
                          (unsigned)sizeof(LoRaPacket));
            // Non-blocking start: Core 0 yields during the ~415 ms air time instead
            // of busy-waiting inside transmit(), which would starve the INT_WDT.
            int state = radio.startTransmit((uint8_t *)&txPkt, (size_t)sizeof(LoRaPacket));
            if (state == RADIOLIB_ERR_NONE) {
                // Block in a yield-able wait for the TX-done DIO1 IRQ (1 s >> air time)
                if (xSemaphoreTake(xLoRaIrqSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
                    state = radio.finishTransmit();
                } else {
                    state = -1;  // TX-done IRQ never fired
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
            int rxState = radio.startReceive();
            if (rxState != RADIOLIB_ERR_NONE) {
                Serial.printf("[LoRa] startReceive after TX failed: %d\n", rxState);
            }
        }

        // Wait up to 10 ms for RX interrupt
        if (xSemaphoreTake(xLoRaIrqSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            int rxLen = radio.getPacketLength();

            if (rxLen == (int)sizeof(LoRaPacket)) {
                int state = radio.readData(rxBuf, (size_t)sizeof(LoRaPacket));
                if (state == RADIOLIB_ERR_NONE) {
                    int8_t rssi = (int8_t)radio.getRSSI();
                    int8_t snr  = (int8_t)radio.getSNR();

                    LoRaPacket *pkt = reinterpret_cast<LoRaPacket *>(rxBuf);

                    if (pkt->header.magic_word != MY_NETWORK_MAGIC ||
                        (pkt->header.target_id  != NODE_TANK_LOCAL &&
                         pkt->header.target_id  != NODE_BROADCAST)) {
                        radio.startReceive();
                        continue;
                    }

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
                    Serial.printf("[LoRa] RX data error: %d\n", state);
                }
            } else if (rxLen > 0) {
                radio.readData(rxBuf, (size_t)min(rxLen, (int)sizeof(rxBuf)));
            }
            int rxState = radio.startReceive();
            if (rxState != RADIOLIB_ERR_NONE) {
                Serial.printf("[LoRa] startReceive after RX failed: %d\n", rxState);
            }
        }
    }
}

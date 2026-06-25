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
            int state = radio.transmit((uint8_t *)&txPkt, (size_t)sizeof(LoRaPacket));
            if (state == RADIOLIB_ERR_NONE) {
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    gState.txFlash        = true;
                    gState.txFlashTime_ms = millis();
                    xSemaphoreGive(xStateMutex);
                }
            } else {
                Serial.printf("[LoRa] TX error: %d\n", state);
            }
            radio.startReceive();
        }

        // Wait up to 10 ms for RX interrupt
        if (xSemaphoreTake(xLoRaIrqSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            int rxLen = radio.getPacketLength();

            if (rxLen == (int)sizeof(LoRaPacket)) {
                int state = radio.readData(rxBuf, (size_t)sizeof(LoRaPacket));
                if (state == RADIOLIB_ERR_NONE) {
                    // Capture signal quality immediately after readData()
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
            radio.startReceive();
        }
    }
}

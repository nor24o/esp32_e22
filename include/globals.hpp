#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <FastLED.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "config.hpp"
#include "protocol.hpp"

// =============================================================================
// GLOBAL SYSTEM STATE (mutex-protected)
// =============================================================================

struct SystemState {
    uint8_t  waterLevel;
    bool     autoMode;
    bool     relay_p1, relay_p2, relay_p3;
    bool     pondPump;
    uint8_t  errorCode;
    bool     awaitingAck;
    uint32_t lastGatewayContact_ms;
    uint32_t lastPondContact_ms;
    float    temperature;
    float    humidity;
    bool     btnMode_edge, btnP1_edge, btnP2_edge, btnP3_edge, btnPond_edge;
    bool     rxFlash, txFlash;
    uint32_t rxFlashTime_ms, txFlashTime_ms;
    int8_t   lastRssi;
    int8_t   lastSnr;
};

static SystemState gState;
static bool        gRadioOk = false;  // set true only after successful radio.begin()+startReceive()
static bool        gPcfOk   = false;  // set true only after PCF8574 ACKs its address in setup()

// =============================================================================
// FREERTOS SYNCHRONISATION HANDLES
// =============================================================================

static SemaphoreHandle_t xStateMutex          = nullptr;
static SemaphoreHandle_t xLoRaIrqSemaphore    = nullptr;
static SemaphoreHandle_t xI2cMutex            = nullptr;
static QueueHandle_t     xTxQueue             = nullptr;
static QueueHandle_t     xRxQueue             = nullptr;
static TaskHandle_t      xControlEngineTask   = nullptr;

// =============================================================================
// PERIPHERAL INSTANCES
// =============================================================================

#ifdef CONFIG_IDF_TARGET_ESP32S3
static SPIClass loraSPI(FSPI);   // SPI2 on ESP32-S3
#else
static SPIClass loraSPI(VSPI);   // VSPI on classic ESP32
#endif
static SX1262   radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, loraSPI);
static CRGB     leds[NUM_LEDS];

// =============================================================================
// REPLAY-ATTACK SEQUENCE TRACKING
// =============================================================================

static uint8_t gOutMsgId          = 0;
static uint8_t gLastMsgId_Gateway = 0;
static uint8_t gLastMsgId_Pond    = 0;
static bool    gMsgIdInit_Gateway = false;
static bool    gMsgIdInit_Pond    = false;

static inline uint8_t nextMsgId() { return ++gOutMsgId; }

static bool isValidMsgId(uint8_t sender_id, uint8_t incoming_id) {
    if (sender_id == NODE_GATEWAY) {
        if (!gMsgIdInit_Gateway) {
            gMsgIdInit_Gateway = true; gLastMsgId_Gateway = incoming_id; return true;
        }
        if ((int8_t)(incoming_id - gLastMsgId_Gateway) > 0) {
            gLastMsgId_Gateway = incoming_id; return true;
        }
        return false;
    }
    if (sender_id == NODE_POND_REMOTE) {
        if (!gMsgIdInit_Pond) {
            gMsgIdInit_Pond = true; gLastMsgId_Pond = incoming_id; return true;
        }
        if ((int8_t)(incoming_id - gLastMsgId_Pond) > 0) {
            gLastMsgId_Pond = incoming_id; return true;
        }
        return false;
    }
    return true;
}

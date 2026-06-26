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
// GLOBAL SYSTEM STATE
//
// All tasks share this one struct.  Any access must be protected by
// xStateMutex to prevent one task reading half-written data from another.
// =============================================================================

struct SystemState {
    // -- Water level and operating mode --
    uint8_t  waterLevel;     // 0 = empty, 1/2/3 = float switches triggered

    bool     autoMode;       // true = system controls pond pump automatically

    // -- Pump states (true = running) --
    bool     relay_p1;
    bool     relay_p2;
    bool     relay_p3;
    bool     pondPump;       // What we believe the pond pump state is

    // -- Errors and diagnostics --
    uint8_t  errorCode;      // 0 = OK,  3 = lost contact with a peer
    bool     pondCmdPending; // true while waiting for pond telemetry to confirm the last command

    // -- When we last heard from each peer (used to detect lost comms) --
    uint32_t lastGatewayContact_ms;
    uint32_t lastPondContact_ms;

    // -- Environment sensors --
    float    temperature;
    float    humidity;

    // -- Touch button edges (set by InputSensorPoll, cleared by ControlEngine) --
    // An "edge" means: the button was just pressed (not held).
    bool     btnMode_edge;
    bool     btnP1_edge;
    bool     btnP2_edge;
    bool     btnP3_edge;
    bool     btnPond_edge;

    // -- LED flash triggers (set by LoRaTransceiver, cleared by UIAnimation) --
    bool     rxFlash;
    bool     txFlash;
    uint32_t rxFlashTime_ms;
    uint32_t txFlashTime_ms;

    // -- Last received radio signal quality --
    int8_t   lastRssi;   // dBm — more negative = weaker signal
    int8_t   lastSnr;    // dB  — higher = cleaner signal
};

// Single shared instance, only accessed through the mutex above.
static SystemState gState;

// Set to true once the radio is confirmed working (after radio.begin() succeeds).
// While false, all LoRa operations are skipped.
static bool gRadioOk = false;

// Set to true once the PCF8574 I/O expander acknowledges its I2C address.
// While false, float switches and touch buttons report default "no water, no press" states.
static bool gPcfOk   = false;

// =============================================================================
// FREERTOS HANDLES
// =============================================================================

static SemaphoreHandle_t xStateMutex          = nullptr;  // Protects gState
static SemaphoreHandle_t xLoRaIrqSemaphore    = nullptr;  // Signals radio interrupt to LoRaTransceiver task
static SemaphoreHandle_t xI2cMutex            = nullptr;  // Protects the I2C bus (shared by PCF8574 task)
static QueueHandle_t     xTxQueue             = nullptr;  // Packets waiting to be transmitted
static QueueHandle_t     xRxQueue             = nullptr;  // Packets received, waiting for ControlEngine
static TaskHandle_t      xControlEngineTask   = nullptr;  // Used to wake ControlEngine after new RX packet

// =============================================================================
// PERIPHERAL INSTANCES
// =============================================================================

#ifdef CONFIG_IDF_TARGET_ESP32S3
static SPIClass loraSPI(FSPI);
#else
static SPIClass loraSPI(VSPI);
#endif
static SX1262   radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, loraSPI);
static CRGB     leds[NUM_LEDS];

// =============================================================================
// DUPLICATE PACKET DETECTION
//
// Each sender increments its msg_id with every packet.  If we receive a
// msg_id that is not newer than the last one we saw, we discard the packet.
// This protects against the radio delivering the same packet twice (which
// can happen with LoRa at the fringe of range).
// =============================================================================

static uint8_t gOutMsgId          = 0;
static uint8_t gLastMsgId_Gateway = 0;
static uint8_t gLastMsgId_Pond    = 0;
static bool    gMsgIdInit_Gateway = false;
static bool    gMsgIdInit_Pond    = false;

static inline uint8_t nextMsgId() { return ++gOutMsgId; }

// Returns true if the incoming msg_id is genuinely newer than the last seen.
// Uses signed arithmetic so the 0→255 wraparound is handled correctly.
static bool isValidMsgId(uint8_t sender_id, uint8_t incoming_id) {
    if (sender_id == NODE_GATEWAY) {
        if (!gMsgIdInit_Gateway) {
            gMsgIdInit_Gateway = true;
            gLastMsgId_Gateway = incoming_id;
            return true;
        }
        if ((int8_t)(incoming_id - gLastMsgId_Gateway) > 0) {
            gLastMsgId_Gateway = incoming_id;
            return true;
        }
        return false;  // same or older — duplicate, discard
    }
    if (sender_id == NODE_POND_REMOTE) {
        if (!gMsgIdInit_Pond) {
            gMsgIdInit_Pond = true;
            gLastMsgId_Pond = incoming_id;
            return true;
        }
        if ((int8_t)(incoming_id - gLastMsgId_Pond) > 0) {
            gLastMsgId_Pond = incoming_id;
            return true;
        }
        return false;
    }
    return true;  // unknown sender — accept (no tracking)
}

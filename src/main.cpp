// =============================================================================
// TANK CONTROLLER NODE (0x02)  –  Production Firmware v1.0
// Target  : Espressif ESP32 (Xtensa LX6 Dual-Core)
// Framework: Arduino + FreeRTOS
// Libraries: RadioLib (SX1262), FastLED (WS2812B), Bounce2, DHT22
//
// HARDWARE NOTES:
//   • GPIO 34-39 are input-only on ESP32-WROOM-32; they do NOT support
//     internal pull-ups.  Install 10 kΩ external pull-ups to 3.3 V on all
//     float-switch lines before these pins.
//   • CURRENT_P2_ADC (GPIO 0) is a strapping pin.  Ensure the ADC input
//     signal is < 0.6 V at power-on to prevent boot failure.  Alternatively
//     re-assign to an I2C ADC (ADS1115) and adjust readCurrentADC().
//   • CURRENT_P3_ADC (GPIO 3) shares the UART0-RX line.  Disable the Serial
//     monitor before using it for current sensing in production.
//   • RadioLib transmit() is blocking.  Its maximum blocking time is bounded
//     by the configured SF/BW; at SF9/125 kHz a max-payload frame is ~330 ms.
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <FastLED.h>
#include <Bounce2.h>
#include <DHT.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <stdint.h>
#include <string.h>

// =============================================================================
// SECTION 1 – PIN DEFINITIONS
// =============================================================================

// SX1262 LoRa via VSPI
#define LORA_NSS   5
#define LORA_DIO1  26
#define LORA_NRST  14
#define LORA_BUSY  27
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23

// Addressable LED strip
#define WS2812B_PIN  12
#define NUM_LEDS     10

// Button inputs (active-low, internal pull-up)
#define BTN_MODE  32
#define BTN_P1    33
#define BTN_P2    25
#define BTN_P3    21
#define BTN_POND  22

// Float-switch inputs (active-low, EXTERNAL pull-up required – GPIO 34-39)
#define FLOAT_0  34   // lowest level
#define FLOAT_1  35
#define FLOAT_2  36
#define FLOAT_3  39   // highest level

// Relay outputs (HIGH = energised)
#define RELAY_P1  13
#define RELAY_P2  15
#define RELAY_P3  2

// Analog current-sense inputs (see hardware notes above)
#define CURRENT_P1_ADC  4    // ADC2_CH0 – safest free pin
#define CURRENT_P2_ADC  0    // ADC2_CH1 – strapping pin, verify hardware
#define CURRENT_P3_ADC  3    // UART0_RX – disable serial in production

// DHT22 ambient sensor
#define DHT_PIN   16
#define DHT_TYPE  DHT22

// =============================================================================
// SECTION 2 – COMPILE-TIME CONSTANTS
// =============================================================================

// LoRa RF parameters
#define LORA_FREQUENCY   433.0f   // MHz – adjust to regional band plan
#define LORA_BANDWIDTH   125.0f   // kHz
#define LORA_SF          9
#define LORA_CR          5        // 4/5 coding rate
#define LORA_SYNC_WORD   0x12    // Private-network sync word
#define LORA_TX_POWER    22      // dBm
#define LORA_PREAMBLE    8

// Network identifiers
#define MY_NETWORK_MAGIC  0x5A6B
#define NODE_GATEWAY      0x01
#define NODE_TANK_LOCAL   0x02
#define NODE_POND_REMOTE  0x03
#define NODE_BROADCAST    0xFF

// Current-sense ADC thresholds (12-bit, 0-4095; calibrate per shunt/amplifier)
#define CURRENT_OVERCURRENT_THRESH  3200   // > this raw ADC = overcurrent / stall
#define CURRENT_DRYRUN_THRESH        150   // < this raw ADC when relay HIGH = dry run

// Timing (all in milliseconds)
#define PUMP_MIN_RUNTIME_MS     30000UL   // 30 s minimum ON per cycle
#define PUMP_MIN_COOLDOWN_MS    60000UL   // 60 s minimum OFF between cycles
#define REPLENISH_RUNON_MS     300000UL   // 5 min minimum pond-fill lock-on
#define ACK_TIMEOUT_MS           5000UL   // wait 5 s for ACK before retry
#define ACK_MAX_RETRIES             3     // attempts before declaring comms fault
#define NETWORK_TIMEOUT_MS       60000UL  // 60 s without contact = timeout error
#define TELEMETRY_INTERVAL_MS    30000UL  // periodic uplink to gateway
#define DHT_READ_INTERVAL_MS      2000UL  // DHT22 minimum sample interval

// ADC averaging
#define ADC_SAMPLES  8

// LED flash durations
#define FLASH_RX_MS   200UL
#define FLASH_TX_MS   200UL
#define FLASH_NET_MS  400UL   // half-period for network-timeout flash

// =============================================================================
// SECTION 3 – PACKED NETWORK PROTOCOL STRUCTURES
// =============================================================================

enum MessageType : uint8_t {
    MSG_TELEMETRY = 1,
    MSG_COMMAND   = 2,
    MSG_ACK       = 3
};

struct __attribute__((packed)) LoRaHeader {
    uint16_t magic_word;   // Must equal MY_NETWORK_MAGIC
    uint8_t  target_id;
    uint8_t  sender_id;
    uint8_t  msg_id;       // Sequence counter – uint8 wraps intentionally
    uint8_t  msg_type;     // MessageType enum
};

struct __attribute__((packed)) TelemetryData {
    uint8_t  water_level;   // 0–4
    uint8_t  system_flags;  // bit0=Auto, bit1=P1, bit2=P2, bit3=P3, bit4=PondP
    int16_t  temperature;   // °C × 10  (e.g. 255 = 25.5 °C)
    uint8_t  humidity;      // % RH, rounded
    uint8_t  error_code;    // 0=OK 1=Overcurrent 2=DryRun 3=CommsTimeout
};

struct __attribute__((packed)) CommandData {
    uint8_t target_pump;    // 1=P1, 2=P2, 3=P3, 4=PondPump; reused as acked-msg-id in ACK frames
    uint8_t action;         // 0=Off, 1=On
    uint8_t flags;          // bit0=Force Manual Override
};

struct __attribute__((packed)) LoRaPacket {
    LoRaHeader header;
    union {
        TelemetryData telemetry;
        CommandData   command;    // also used for ACK payload (target_pump = acked msg_id)
    } payload;
};

static_assert(sizeof(LoRaPacket) <= 255, "LoRaPacket exceeds max RadioLib payload");

// =============================================================================
// SECTION 4 – GLOBAL SYSTEM-STATE (mutex-protected)
// =============================================================================

struct SystemState {
    // Water
    uint8_t  waterLevel;             // 0–4 (debounced)

    // Operation
    bool     autoMode;

    // Relay / pump states
    bool     relay_p1;
    bool     relay_p2;
    bool     relay_p3;
    bool     pondPump;               // Commanded state of remote pond pump

    // Fault
    uint8_t  errorCode;              // 0=OK 1=OC 2=DR 3=Comms
    bool     faultLockout;           // Latched on HW fault, cleared by MODE btn

    // Network comms
    bool     awaitingAck;
    uint32_t lastGatewayContact_ms;
    uint32_t lastPondContact_ms;

    // Climate
    float    temperature;
    float    humidity;

    // Button edge flags (set by InputSensor, consumed by ControlEngine)
    bool     btnMode_edge;
    bool     btnP1_edge;
    bool     btnP2_edge;
    bool     btnP3_edge;
    bool     btnPond_edge;

    // Current readings (averaged raw ADC)
    uint16_t currentP1;
    uint16_t currentP2;
    uint16_t currentP3;

    // UI flash event flags (set by tasks, consumed by UIAnimation)
    bool     rxFlash;
    bool     txFlash;
    uint32_t rxFlashTime_ms;
    uint32_t txFlashTime_ms;
};

static SystemState gState;

// =============================================================================
// SECTION 5 – FREERTOS SYNCHRONISATION HANDLES
// =============================================================================

static SemaphoreHandle_t xStateMutex        = nullptr;
static SemaphoreHandle_t xLoRaIrqSemaphore  = nullptr;
static QueueHandle_t     xTxQueue           = nullptr;  // ControlEngine → LoRa TX
static QueueHandle_t     xRxQueue           = nullptr;  // LoRa RX  → ControlEngine

// =============================================================================
// SECTION 6 – PERIPHERAL INSTANCES
// =============================================================================

static SPIClass loraSPI(VSPI);
static SX1262   radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, loraSPI);
static CRGB     leds[NUM_LEDS];
static DHT      dht(DHT_PIN, DHT_TYPE);
static Bounce   floatSwitch[4];
static Bounce   btn[5];   // 0=MODE 1=P1 2=P2 3=P3 4=POND

// =============================================================================
// SECTION 7 – REPLAY-ATTACK SEQUENCE TRACKING
// =============================================================================

static uint8_t gOutMsgId = 0;           // This node's outgoing counter

static uint8_t gLastMsgId_Gateway  = 0; // Last accepted msg_id from gateway
static uint8_t gLastMsgId_Pond     = 0; // Last accepted msg_id from pond
static bool    gMsgIdInit_Gateway  = false;
static bool    gMsgIdInit_Pond     = false;

// Returns the next outgoing message-id (wraps at 255 → 0 intentionally)
static inline uint8_t nextMsgId() { return ++gOutMsgId; }

// Returns true and updates tracker if the incoming id is strictly newer.
// Uses signed subtraction for correct uint8_t wraparound detection.
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
        return false;
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
    return true; // Unknown senders pass through untracked
}

// =============================================================================
// SECTION 8 – RADIOLIB DIO1 INTERRUPT SERVICE ROUTINE
// =============================================================================

void IRAM_ATTR onPacketReceived() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xLoRaIrqSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// =============================================================================
// SECTION 9 – SHARED HELPER FUNCTIONS
// =============================================================================

static uint16_t readCurrentADC(uint8_t pin) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < ADC_SAMPLES; i++) sum += (uint32_t)analogRead(pin);
    return (uint16_t)(sum / ADC_SAMPLES);
}

// Queue a telemetry packet from a snapshot of the current global state.
// Safe to call from any task; acquires the mutex internally.
static void buildAndQueueTelemetry() {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.header.magic_word = MY_NETWORK_MAGIC;
    pkt.header.target_id  = NODE_GATEWAY;
    pkt.header.sender_id  = NODE_TANK_LOCAL;
    pkt.header.msg_id     = nextMsgId();
    pkt.header.msg_type   = MSG_TELEMETRY;

    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        uint8_t flags = 0;
        if (gState.autoMode)  flags |= (1 << 0);
        if (gState.relay_p1)  flags |= (1 << 1);
        if (gState.relay_p2)  flags |= (1 << 2);
        if (gState.relay_p3)  flags |= (1 << 3);
        if (gState.pondPump)  flags |= (1 << 4);
        pkt.payload.telemetry.water_level  = gState.waterLevel;
        pkt.payload.telemetry.system_flags = flags;
        pkt.payload.telemetry.temperature  = (int16_t)(gState.temperature * 10.0f);
        pkt.payload.telemetry.humidity     = (uint8_t)(gState.humidity + 0.5f);
        pkt.payload.telemetry.error_code   = gState.errorCode;
        xSemaphoreGive(xStateMutex);
    }
    xQueueSend(xTxQueue, &pkt, 0);
}

// Build and queue a pump command directed at a remote node.
// Returns the msg_id assigned so the caller can track ACKs.
static uint8_t sendPumpCommand(uint8_t target_node, uint8_t pump_id, uint8_t action) {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.header.magic_word       = MY_NETWORK_MAGIC;
    pkt.header.target_id        = target_node;
    pkt.header.sender_id        = NODE_TANK_LOCAL;
    pkt.header.msg_id           = nextMsgId();
    pkt.header.msg_type         = MSG_COMMAND;
    pkt.payload.command.target_pump = pump_id;
    pkt.payload.command.action      = action;
    pkt.payload.command.flags       = 0;
    xQueueSend(xTxQueue, &pkt, 0);
    return pkt.header.msg_id;
}

// Queue an ACK back to a sender.
static void sendAck(uint8_t target_id, uint8_t acked_msg_id) {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.header.magic_word           = MY_NETWORK_MAGIC;
    pkt.header.target_id            = target_id;
    pkt.header.sender_id            = NODE_TANK_LOCAL;
    pkt.header.msg_id               = nextMsgId();
    pkt.header.msg_type             = MSG_ACK;
    pkt.payload.command.target_pump = acked_msg_id; // carries the id being acknowledged
    pkt.payload.command.action      = 0;
    pkt.payload.command.flags       = 0;
    xQueueSend(xTxQueue, &pkt, 0);
}

// =============================================================================
// SECTION 10 – PUMP HYSTERESIS MANAGEMENT
// =============================================================================

struct PumpHysteresis {
    bool     state;
    uint32_t lastOnTime_ms;
    uint32_t lastOffTime_ms;
    bool     initialized;
};

static bool canTurnPumpOn(const PumpHysteresis &ph) {
    if (!ph.initialized) return true;
    if (ph.state)         return false;  // Already on
    return (millis() - ph.lastOffTime_ms) >= PUMP_MIN_COOLDOWN_MS;
}

static bool canTurnPumpOff(const PumpHysteresis &ph) {
    if (!ph.initialized) return true;
    if (!ph.state)        return false;  // Already off
    return (millis() - ph.lastOnTime_ms) >= PUMP_MIN_RUNTIME_MS;
}

// Drive a physical relay and update the hysteresis tracker.
// Pass relay_pin = -1 for remote pumps (no local GPIO to drive).
static void setPumpState(PumpHysteresis &ph, bool newState, int relay_pin) {
    if (ph.initialized && (newState == ph.state)) return;
    if (relay_pin >= 0) digitalWrite((uint8_t)relay_pin, newState ? HIGH : LOW);
    if (newState) ph.lastOnTime_ms  = millis();
    else          ph.lastOffTime_ms = millis();
    ph.state       = newState;
    ph.initialized = true;
}

// =============================================================================
// SECTION 11 – TASK: INPUT SENSOR POLL  (Priority 3, Core 1, 20 ms period)
// =============================================================================

static void Task_InputSensorPoll(void *pvParams) {
    const uint8_t floatPins[4] = { FLOAT_0, FLOAT_1, FLOAT_2, FLOAT_3 };
    const uint8_t btnPins[5]   = { BTN_MODE, BTN_P1, BTN_P2, BTN_P3, BTN_POND };
    const uint8_t curPins[3]   = { CURRENT_P1_ADC, CURRENT_P2_ADC, CURRENT_P3_ADC };

    // Float switches – 2 000 ms debounce; external pull-up, no internal pull-up
    for (uint8_t i = 0; i < 4; i++) {
        floatSwitch[i].attach(floatPins[i], INPUT);
        floatSwitch[i].interval(2000);
    }

    // Buttons – 25 ms debounce; active-low with internal pull-up
    for (uint8_t i = 0; i < 5; i++) {
        btn[i].attach(btnPins[i], INPUT_PULLUP);
        btn[i].interval(25);
    }

    uint32_t dhtLastRead_ms = 0;
    float    lastTemp       = 0.0f;
    float    lastHumid      = 0.0f;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));

        // --- Update all debounce instances ---
        for (uint8_t i = 0; i < 4; i++) floatSwitch[i].update();
        for (uint8_t i = 0; i < 5; i++) btn[i].update();

        // --- Water level: count contiguous submerged floats from bottom ---
        // LOW = water present (external pull-up to 3.3 V, switch closes to GND)
        uint8_t level = 0;
        for (uint8_t i = 0; i < 4; i++) {
            if (floatSwitch[i].read() == LOW) {
                level = (uint8_t)(i + 1);
            } else {
                break;  // Gap in sensor stack terminates count
            }
        }

        // --- Averaged current readings ---
        uint16_t cur[3];
        for (uint8_t i = 0; i < 3; i++) cur[i] = readCurrentADC(curPins[i]);

        // --- DHT22 (sampled at 2 s intervals to respect sensor timing) ---
        uint32_t now = millis();
        if ((now - dhtLastRead_ms) >= DHT_READ_INTERVAL_MS) {
            dhtLastRead_ms = now;
            float t = dht.readTemperature();
            float h = dht.readHumidity();
            if (!isnan(t) && !isnan(h)) {
                lastTemp  = t;
                lastHumid = h;
            }
        }

        // --- Button fell() = active-low press detected after debounce ---
        bool edges[5];
        for (uint8_t i = 0; i < 5; i++) edges[i] = btn[i].fell();

        // --- Write verified state to global struct under mutex ---
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            gState.waterLevel  = level;
            gState.currentP1   = cur[0];
            gState.currentP2   = cur[1];
            gState.currentP3   = cur[2];
            gState.temperature = lastTemp;
            gState.humidity    = lastHumid;
            // OR-in edges so a fast button tap is never lost between 50 ms polls
            if (edges[0]) gState.btnMode_edge = true;
            if (edges[1]) gState.btnP1_edge   = true;
            if (edges[2]) gState.btnP2_edge   = true;
            if (edges[3]) gState.btnP3_edge   = true;
            if (edges[4]) gState.btnPond_edge  = true;
            xSemaphoreGive(xStateMutex);
        }
    }
}

// =============================================================================
// SECTION 12 – TASK: CONTROL ENGINE  (Priority 2, Core 1, 50 ms period)
// =============================================================================

static void Task_ControlEngine(void *pvParams) {
    PumpHysteresis ph_p1   = { false, 0, 0, false };
    PumpHysteresis ph_p2   = { false, 0, 0, false };
    PumpHysteresis ph_p3   = { false, 0, 0, false };
    PumpHysteresis ph_pond = { false, 0, 0, false };

    bool     replenishActive    = false;
    uint32_t replenishStartTime = 0;

    // ACK-wait state for outgoing pond commands
    bool     awaitingAckInternal = false;
    uint32_t ackSentTime_ms      = 0;
    uint8_t  ackRetryCount       = 0;
    uint8_t  pendingPondAction   = 0;   // 0=OFF 1=ON – needed for retries

    uint32_t lastTelemetryTime_ms = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));

        uint32_t now = millis();

        // ── Snapshot global state ────────────────────────────────────────────
        uint8_t  waterLevel;
        bool     autoMode, faultLockout;
        uint8_t  errorCode;
        uint16_t curP1, curP2, curP3;
        uint32_t lastGateway, lastPond;
        bool     btnMode, btnP1, btnP2, btnP3, btnPond;

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) != pdTRUE) continue;

        waterLevel   = gState.waterLevel;
        autoMode     = gState.autoMode;
        faultLockout = gState.faultLockout;
        errorCode    = gState.errorCode;
        curP1        = gState.currentP1;
        curP2        = gState.currentP2;
        curP3        = gState.currentP3;
        lastGateway  = gState.lastGatewayContact_ms;
        lastPond     = gState.lastPondContact_ms;
        // Consume edge flags atomically
        btnMode = gState.btnMode_edge; gState.btnMode_edge = false;
        btnP1   = gState.btnP1_edge;   gState.btnP1_edge   = false;
        btnP2   = gState.btnP2_edge;   gState.btnP2_edge   = false;
        btnP3   = gState.btnP3_edge;   gState.btnP3_edge   = false;
        btnPond = gState.btnPond_edge;  gState.btnPond_edge  = false;

        xSemaphoreGive(xStateMutex);

        // ── Network-timeout assessment ───────────────────────────────────────
        bool networkOk = ((now - lastGateway) < NETWORK_TIMEOUT_MS) &&
                         ((now - lastPond)    < NETWORK_TIMEOUT_MS);

        // ── Overcurrent / Dry-run protection ─────────────────────────────────
        bool overCurrent = (ph_p1.state && curP1 > CURRENT_OVERCURRENT_THRESH) ||
                           (ph_p2.state && curP2 > CURRENT_OVERCURRENT_THRESH) ||
                           (ph_p3.state && curP3 > CURRENT_OVERCURRENT_THRESH);

        bool dryRun = (ph_p1.state && curP1 < CURRENT_DRYRUN_THRESH) ||
                      (ph_p2.state && curP2 < CURRENT_DRYRUN_THRESH) ||
                      (ph_p3.state && curP3 < CURRENT_DRYRUN_THRESH);

        if (overCurrent || dryRun) {
            // Immediate hardware kill – bypass hysteresis timers
            digitalWrite(RELAY_P1, LOW);
            digitalWrite(RELAY_P2, LOW);
            digitalWrite(RELAY_P3, LOW);
            ph_p1.state = false; ph_p1.lastOffTime_ms = now;
            ph_p2.state = false; ph_p2.lastOffTime_ms = now;
            ph_p3.state = false; ph_p3.lastOffTime_ms = now;

            uint8_t faultCode = overCurrent ? 1 : 2;

            if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                gState.relay_p1     = false;
                gState.relay_p2     = false;
                gState.relay_p3     = false;
                gState.errorCode    = faultCode;
                gState.faultLockout = true;
                xSemaphoreGive(xStateMutex);
            }
            buildAndQueueTelemetry();
            continue;   // Skip all further control logic this cycle
        }

        // ── Fault-lockout gate ───────────────────────────────────────────────
        if (faultLockout) {
            // Single press of MODE button clears a latched fault
            if (btnMode) {
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    gState.faultLockout = false;
                    gState.errorCode    = 0;
                    xSemaphoreGive(xStateMutex);
                }
            }
            // Still report network timeout even while locked out
            if (!networkOk) {
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    if (gState.errorCode == 0) gState.errorCode = 3;
                    xSemaphoreGive(xStateMutex);
                }
            }
            continue;
        }

        // ── Mode toggle ──────────────────────────────────────────────────────
        if (btnMode) {
            autoMode = !autoMode;
            if (!autoMode && replenishActive) {
                // Cancel autonomous replenishment when dropping to manual
                replenishActive = false;
            }
            if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                gState.autoMode = autoMode;
                xSemaphoreGive(xStateMutex);
            }
        }

        // ── Process received LoRa packets from the RX queue ──────────────────
        LoRaPacket rxPkt;
        while (xQueueReceive(xRxQueue, &rxPkt, 0) == pdTRUE) {

            if (rxPkt.header.msg_type == MSG_COMMAND) {
                if (!isValidMsgId(rxPkt.header.sender_id, rxPkt.header.msg_id)) {
                    continue;   // Replay detected – silently drop
                }
                CommandData &cmd = rxPkt.payload.command;
                bool forceOverride = (cmd.flags & 0x01) != 0;

                // In Auto mode, reject commands unless the gateway forces an override
                if (autoMode && !forceOverride) {
                    sendAck(rxPkt.header.sender_id, rxPkt.header.msg_id);
                    continue;
                }

                // Execute command, respecting hysteresis on every pump
                switch (cmd.target_pump) {
                    case 1:
                        if (cmd.action == 1 && canTurnPumpOn(ph_p1))
                            setPumpState(ph_p1, true,  RELAY_P1);
                        else if (cmd.action == 0 && canTurnPumpOff(ph_p1))
                            setPumpState(ph_p1, false, RELAY_P1);
                        break;
                    case 2:
                        if (cmd.action == 1 && canTurnPumpOn(ph_p2))
                            setPumpState(ph_p2, true,  RELAY_P2);
                        else if (cmd.action == 0 && canTurnPumpOff(ph_p2))
                            setPumpState(ph_p2, false, RELAY_P2);
                        break;
                    case 3:
                        if (cmd.action == 1 && canTurnPumpOn(ph_p3))
                            setPumpState(ph_p3, true,  RELAY_P3);
                        else if (cmd.action == 0 && canTurnPumpOff(ph_p3))
                            setPumpState(ph_p3, false, RELAY_P3);
                        break;
                    default:
                        break;
                }

                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    gState.relay_p1 = ph_p1.state;
                    gState.relay_p2 = ph_p2.state;
                    gState.relay_p3 = ph_p3.state;
                    if (rxPkt.header.sender_id == NODE_GATEWAY)
                        gState.lastGatewayContact_ms = now;
                    xSemaphoreGive(xStateMutex);
                }
                sendAck(rxPkt.header.sender_id, rxPkt.header.msg_id);

            } else if (rxPkt.header.msg_type == MSG_ACK) {
                if (rxPkt.header.sender_id == NODE_POND_REMOTE) {
                    // Any ACK from pond clears our outstanding wait
                    awaitingAckInternal = false;
                    ackRetryCount       = 0;
                    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        gState.awaitingAck        = false;
                        gState.lastPondContact_ms = now;
                        xSemaphoreGive(xStateMutex);
                    }
                } else if (rxPkt.header.sender_id == NODE_GATEWAY) {
                    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        gState.lastGatewayContact_ms = now;
                        xSemaphoreGive(xStateMutex);
                    }
                }

            } else if (rxPkt.header.msg_type == MSG_TELEMETRY) {
                if (rxPkt.header.sender_id == NODE_POND_REMOTE) {
                    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        gState.lastPondContact_ms = now;
                        xSemaphoreGive(xStateMutex);
                    }
                }
            }
        }

        // ── ACK timeout / retry for outgoing pond commands ───────────────────
        if (awaitingAckInternal && (now - ackSentTime_ms) >= ACK_TIMEOUT_MS) {
            if (ackRetryCount < ACK_MAX_RETRIES) {
                ackRetryCount++;
                ackSentTime_ms = now;
                sendPumpCommand(NODE_POND_REMOTE, 1, pendingPondAction);
            } else {
                awaitingAckInternal = false;
                ackRetryCount       = 0;
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    gState.awaitingAck = false;
                    gState.errorCode   = 3;   // Comms timeout
                    xSemaphoreGive(xStateMutex);
                }
            }
        }

        // ── Update network-timeout error field ───────────────────────────────
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (!networkOk && gState.errorCode == 0) gState.errorCode = 3;
            if ( networkOk && gState.errorCode == 3) gState.errorCode = 0;
            xSemaphoreGive(xStateMutex);
        }

        // ====================================================================
        // AUTO MODE
        // ====================================================================
        if (autoMode) {
            if (waterLevel == 0 && !replenishActive) {
                // Tank empty – start replenishment if hysteresis permits
                if (canTurnPumpOn(ph_pond)) {
                    replenishActive    = true;
                    replenishStartTime = now;
                    setPumpState(ph_pond, true, -1);
                    pendingPondAction   = 1;
                    awaitingAckInternal = true;
                    ackSentTime_ms      = now;
                    ackRetryCount       = 0;
                    sendPumpCommand(NODE_POND_REMOTE, 1, 1);
                    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        gState.pondPump    = true;
                        gState.awaitingAck = true;
                        xSemaphoreGive(xStateMutex);
                    }
                }
            } else if (replenishActive) {
                bool runOnExpired = (now - replenishStartTime) >= REPLENISH_RUNON_MS;
                // Stop only once the mandatory run-on has elapsed AND tank is full
                if (runOnExpired && waterLevel >= 4 && canTurnPumpOff(ph_pond)) {
                    replenishActive = false;
                    setPumpState(ph_pond, false, -1);
                    pendingPondAction   = 0;
                    awaitingAckInternal = true;
                    ackSentTime_ms      = now;
                    ackRetryCount       = 0;
                    sendPumpCommand(NODE_POND_REMOTE, 1, 0);
                    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        gState.pondPump    = false;
                        gState.awaitingAck = true;
                        xSemaphoreGive(xStateMutex);
                    }
                }
            }
        }

        // ====================================================================
        // MANUAL MODE
        // ====================================================================
        if (!autoMode) {
            if (btnP1) {
                bool want = !ph_p1.state;
                if ( want && canTurnPumpOn(ph_p1))  setPumpState(ph_p1, true,  RELAY_P1);
                if (!want && canTurnPumpOff(ph_p1)) setPumpState(ph_p1, false, RELAY_P1);
            }
            if (btnP2) {
                bool want = !ph_p2.state;
                if ( want && canTurnPumpOn(ph_p2))  setPumpState(ph_p2, true,  RELAY_P2);
                if (!want && canTurnPumpOff(ph_p2)) setPumpState(ph_p2, false, RELAY_P2);
            }
            if (btnP3) {
                bool want = !ph_p3.state;
                if ( want && canTurnPumpOn(ph_p3))  setPumpState(ph_p3, true,  RELAY_P3);
                if (!want && canTurnPumpOff(ph_p3)) setPumpState(ph_p3, false, RELAY_P3);
            }
            if (btnPond) {
                bool want = !ph_pond.state;
                if (want && canTurnPumpOn(ph_pond)) {
                    setPumpState(ph_pond, true, -1);
                    pendingPondAction   = 1;
                    awaitingAckInternal = true;
                    ackSentTime_ms      = now;
                    ackRetryCount       = 0;
                    sendPumpCommand(NODE_POND_REMOTE, 1, 1);
                    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        gState.pondPump    = true;
                        gState.awaitingAck = true;
                        xSemaphoreGive(xStateMutex);
                    }
                } else if (!want && canTurnPumpOff(ph_pond)) {
                    setPumpState(ph_pond, false, -1);
                    pendingPondAction   = 0;
                    awaitingAckInternal = true;
                    ackSentTime_ms      = now;
                    ackRetryCount       = 0;
                    sendPumpCommand(NODE_POND_REMOTE, 1, 0);
                    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        gState.pondPump    = false;
                        gState.awaitingAck = true;
                        xSemaphoreGive(xStateMutex);
                    }
                }
            }
        }

        // ── Push relay states back to global state ────────────────────────────
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            gState.relay_p1 = ph_p1.state;
            gState.relay_p2 = ph_p2.state;
            gState.relay_p3 = ph_p3.state;
            gState.pondPump = ph_pond.state;
            xSemaphoreGive(xStateMutex);
        }

        // ── Periodic telemetry uplink ─────────────────────────────────────────
        if ((now - lastTelemetryTime_ms) >= TELEMETRY_INTERVAL_MS) {
            lastTelemetryTime_ms = now;
            buildAndQueueTelemetry();
        }
    }
}

// =============================================================================
// SECTION 13 – TASK: LORA TRANSCEIVER  (Priority 4, Core 0, Event-driven)
// =============================================================================

static void Task_LoRaTransceiver(void *pvParams) {
    uint8_t rxBuf[sizeof(LoRaPacket)];

    for (;;) {
        // ── Outgoing: drain TX queue first (non-blocking peek) ───────────────
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
            // Re-enter continuous receive after every transmission
            radio.startReceive();
        }

        // ── Incoming: wait up to 10 ms for ISR semaphore ─────────────────────
        if (xSemaphoreTake(xLoRaIrqSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            int rxLen = radio.getPacketLength();

            if (rxLen == (int)sizeof(LoRaPacket)) {
                int state = radio.readData(rxBuf, (size_t)sizeof(LoRaPacket));
                if (state == RADIOLIB_ERR_NONE) {
                    LoRaPacket *pkt = reinterpret_cast<LoRaPacket *>(rxBuf);

                    // Hard filter: wrong magic or not addressed to us
                    if (pkt->header.magic_word != MY_NETWORK_MAGIC ||
                        (pkt->header.target_id  != NODE_TANK_LOCAL &&
                         pkt->header.target_id  != NODE_BROADCAST)) {
                        radio.startReceive();
                        continue;
                    }

                    // Pass to control engine; drop silently if queue is full
                    if (xQueueSend(xRxQueue, pkt, 0) == pdTRUE) {
                        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                            gState.rxFlash        = true;
                            gState.rxFlashTime_ms = millis();
                            xSemaphoreGive(xStateMutex);
                        }
                    }
                } else {
                    Serial.printf("[LoRa] RX data error: %d\n", state);
                }
            } else if (rxLen > 0) {
                // Malformed frame – flush the FIFO
                radio.readData(rxBuf, (size_t)min(rxLen, (int)sizeof(rxBuf)));
            }
            radio.startReceive();
        }
    }
}

// =============================================================================
// SECTION 14 – TASK: UI ANIMATION  (Priority 1, Core 0, 30 ms frame rate)
// =============================================================================

static void Task_UIAnimation(void *pvParams) {
    uint8_t  pulsePhase      = 0;
    uint32_t lastRxFlash_ms  = 0;
    uint32_t lastTxFlash_ms  = 0;
    uint32_t netFlashToggle  = 0;
    bool     netFlashOn      = false;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(30));

        uint32_t now = millis();

        // ── Snapshot needed fields ────────────────────────────────────────────
        uint8_t  waterLevel;
        bool     autoMode, faultLockout, awaitingAck;
        uint8_t  errorCode;
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

        // Latch flash start times
        if (rxFlash) lastRxFlash_ms = now;
        if (txFlash) lastTxFlash_ms = now;

        // ── Clear all pixels ─────────────────────────────────────────────────
        fill_solid(leds, NUM_LEDS, CRGB::Black);

        // ── LEDs 0-3: Water level (cyan fill from bottom) ────────────────────
        for (uint8_t i = 0; i < waterLevel && i < 4; i++) {
            leds[i] = CRGB(0, 180, 220);   // Aqua/cyan
        }

        // ── LED 4: System status ──────────────────────────────────────────────
        if (faultLockout || errorCode == 1 || errorCode == 2) {
            // Solid red – local hardware fault
            leds[4] = CRGB::Red;

        } else if (errorCode == 3) {
            // Flash red – network / comms timeout
            if ((now - netFlashToggle) >= FLASH_NET_MS) {
                netFlashToggle = now;
                netFlashOn     = !netFlashOn;
            }
            leds[4] = netFlashOn ? CRGB::Red : CRGB::Black;

        } else if ((now - lastRxFlash_ms) < FLASH_RX_MS) {
            // Flash purple – valid targeted packet received
            leds[4] = CRGB(148, 0, 211);

        } else if ((now - lastTxFlash_ms) < FLASH_TX_MS) {
            // Flash cyan – packet successfully transmitted
            leds[4] = CRGB::Cyan;

        } else if (awaitingAck) {
            // Pulse yellow – awaiting ACK from remote node
            // Integer triangle wave: 256 steps → ~768 ms period at 30 ms/frame
            pulsePhase = (uint8_t)((pulsePhase + 10) & 0xFF);
            uint8_t bri = (pulsePhase < 128)
                          ? (uint8_t)(pulsePhase * 2)
                          : (uint8_t)((255 - pulsePhase) * 2);
            leds[4] = CRGB(bri, bri, 0);

        } else if (autoMode) {
            // Solid green – auto mode / normal
            leds[4] = CRGB::Green;

        } else {
            // Solid blue – manual mode
            leds[4] = CRGB::Blue;
        }

        // ── LED 5: Button-1 / mode indicator ─────────────────────────────────
        leds[5] = autoMode ? CRGB::Green : CRGB::Blue;

        // ── LEDs 6-8: Local relay states (dim red = off, green = on) ─────────
        leds[6] = relay_p1 ? CRGB::Green : CRGB(30, 0, 0);
        leds[7] = relay_p2 ? CRGB::Green : CRGB(30, 0, 0);
        leds[8] = relay_p3 ? CRGB::Green : CRGB(30, 0, 0);

        // ── LED 9: Remote pond pump execution state ───────────────────────────
        leds[9] = pondPump ? CRGB::Cyan : CRGB(0, 0, 30);

        FastLED.show();
    }
}

// =============================================================================
// SECTION 15 – SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("\n[BOOT] Tank Controller Node 0x02 starting...");

    // ── GPIO configuration ────────────────────────────────────────────────────
    pinMode(RELAY_P1, OUTPUT); digitalWrite(RELAY_P1, LOW);
    pinMode(RELAY_P2, OUTPUT); digitalWrite(RELAY_P2, LOW);
    pinMode(RELAY_P3, OUTPUT); digitalWrite(RELAY_P3, LOW);

    analogSetWidth(12);
    analogSetAttenuation(ADC_11db);

    // ── FastLED ───────────────────────────────────────────────────────────────
    FastLED.addLeds<WS2812B, WS2812B_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(80);
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();

    // ── DHT22 ─────────────────────────────────────────────────────────────────
    dht.begin();

    // ── Initialise global state ───────────────────────────────────────────────
    memset(&gState, 0, sizeof(gState));
    gState.autoMode              = true;
    gState.lastGatewayContact_ms = millis();
    gState.lastPondContact_ms    = millis();

    // ── FreeRTOS primitives ───────────────────────────────────────────────────
    xStateMutex = xSemaphoreCreateMutex();
    configASSERT(xStateMutex       != nullptr);

    xLoRaIrqSemaphore = xSemaphoreCreateBinary();
    configASSERT(xLoRaIrqSemaphore != nullptr);

    xTxQueue = xQueueCreate(8, sizeof(LoRaPacket));
    configASSERT(xTxQueue          != nullptr);

    xRxQueue = xQueueCreate(8, sizeof(LoRaPacket));
    configASSERT(xRxQueue          != nullptr);

    // ── RadioLib SX1262 ───────────────────────────────────────────────────────
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    int radioState = radio.begin(
        LORA_FREQUENCY,
        LORA_BANDWIDTH,
        LORA_SF,
        LORA_CR,
        LORA_SYNC_WORD,
        LORA_TX_POWER,
        LORA_PREAMBLE
    );

    if (radioState != RADIOLIB_ERR_NONE) {
        Serial.printf("[FATAL] RadioLib begin() failed: %d\n", radioState);
        fill_solid(leds, NUM_LEDS, CRGB::Red);
        FastLED.show();
        for (;;) vTaskDelay(portMAX_DELAY);   // Halt; watchdog will reset
    }

    radio.setCRC(true);
    radio.setPacketReceivedAction(onPacketReceived);

    radioState = radio.startReceive();
    if (radioState != RADIOLIB_ERR_NONE) {
        Serial.printf("[FATAL] RadioLib startReceive() failed: %d\n", radioState);
        fill_solid(leds, NUM_LEDS, CRGB::Red);
        FastLED.show();
        for (;;) vTaskDelay(portMAX_DELAY);
    }

    Serial.printf("[BOOT] Radio OK  |  Packet size: %u bytes\n",
                  (unsigned)sizeof(LoRaPacket));

    // ── Create FreeRTOS tasks ─────────────────────────────────────────────────
    BaseType_t rc;

    rc = xTaskCreatePinnedToCore(
        Task_InputSensorPoll, "InputPoll",
        4096, nullptr, 3, nullptr, 1);
    configASSERT(rc == pdPASS);

    rc = xTaskCreatePinnedToCore(
        Task_ControlEngine, "CtrlEng",
        8192, nullptr, 2, nullptr, 1);
    configASSERT(rc == pdPASS);

    rc = xTaskCreatePinnedToCore(
        Task_LoRaTransceiver, "LoRaTx",
        8192, nullptr, 4, nullptr, 0);
    configASSERT(rc == pdPASS);

    rc = xTaskCreatePinnedToCore(
        Task_UIAnimation, "UIAnim",
        4096, nullptr, 1, nullptr, 0);
    configASSERT(rc == pdPASS);

    Serial.println("[BOOT] All tasks created.  System operational.");
}

// =============================================================================
// SECTION 16 – LOOP (unused – all work is in RTOS tasks)
// =============================================================================

void loop() {
    vTaskDelay(portMAX_DELAY);
}

// =============================================================================
// TANK CONTROLLER NODE (0x02)  –  Production Firmware v1.1
// Target  : Espressif ESP32 (Xtensa LX6 Dual-Core)
// Framework: Arduino + FreeRTOS
// Libraries: RadioLib (SX1262), FastLED (WS2812B), Adafruit_MCP23X17, Preferences
//
// Protocol v1.1 vs v1.0:
//   • TelemetryData gains last_rssi (int8) and last_snr (int8) — wire-breaking change
//   • MSG_CONFIG_GET/SET/RESP  (types 4/5/6) — runtime config over LoRa
//   • MSG_STATS_GET/RESP       (types 7/8)   — pump run-times, fill cycles, boot info
//   • All timing/threshold constants now live in NVS-backed NodeConfig (gConfig)
//
// HARDWARE NOTES:
//   • MCP23017 I2C expander (0x20): all 5 cap-touch buttons + 3 float switches on GPA0-GPA7.
//   • GPIO 21 = SDA, GPIO 22 = SCL (shared I2C bus; keep traces short, 4.7 kΩ pull-ups to 3.3 V).
//   • GPIO 2  (CURRENT_P3_ADC): strapping pin — sensor output must be < 0.6 V at power-on (pump off = safe).
//   • GPIO 17 (RELAY_P3): safe digital output, no strapping concerns.
//   • RadioLib transmit() blocks; at SF9/125 kHz max payload ≈ 330 ms.
//   • DHT22 removed — temperature/humidity fields in telemetry will be zero.
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RadioLib.h>
#include <FastLED.h>
#include <Adafruit_MCP23X17.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <stdint.h>
#include <string.h>

// =============================================================================
// SECTION 1 – PIN DEFINITIONS
// =============================================================================

#define LORA_NSS   5
#define LORA_DIO1  26
#define LORA_NRST  14
#define LORA_BUSY  27
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23

#define WS2812B_PIN  12   // ⚠ strapping pin (MTDI) — data line must be LOW at power-on
#define NUM_LEDS     10

// I2C bus (shared by MCP23017)
#define I2C_SDA  21
#define I2C_SCL  22

// MCP23017 I2C expander — all buttons and float switches on Port A (GPA0-GPA7)
#define MCP_I2C_ADDR  0x20   // A0=A1=A2=GND → default address

// GPA0-GPA4: capacitive touch buttons (TTP223 drives line actively)
#define MCP_BTN_MODE  0   // GPA0
#define MCP_BTN_P1    1   // GPA1
#define MCP_BTN_P2    2   // GPA2
#define MCP_BTN_P3    3   // GPA3
#define MCP_BTN_POND  4   // GPA4

// GPA5-GPA7: float switches (NC, connect pin to GND when water reaches level)
// MCP23017 internal 100 kΩ pull-up enabled; active = LOW
#define MCP_FLOAT_0   5   // GPA5 – bottom
#define MCP_FLOAT_1   6   // GPA6 – mid
#define MCP_FLOAT_2   7   // GPA7 – top

#define RELAY_P1  13
#define RELAY_P2  15
#define RELAY_P3  17  // was GPIO 2 (strapping pin) → moved to safe GPIO 17

#define CURRENT_P1_ADC  4   // ADC2_CH0
#define CURRENT_P2_ADC  39  // ADC1_CH3 – input-only, safe
#define CURRENT_P3_ADC  2   // ADC2_CH2  ⚠ strapping pin — sensor out must be <0.6V at boot

// =============================================================================
// SECTION 2 – COMPILE-TIME CONSTANTS AND DEFAULTS
// Items marked DEF_* seed NVS on first boot; after that gConfig is authoritative.
// =============================================================================

// LoRa RF  (fixed – not runtime-configurable)
#define LORA_FREQUENCY   868.0f
#define LORA_BANDWIDTH   125.0f
#define LORA_SF          9
#define LORA_CR          5
#define LORA_SYNC_WORD   0x12
#define LORA_TX_POWER    22
#define LORA_PREAMBLE    8

// Network identifiers
#define MY_NETWORK_MAGIC  0x5A6B
#define NODE_GATEWAY      0x01
#define NODE_TANK_LOCAL   0x02
#define NODE_POND_REMOTE  0x03
#define NODE_BROADCAST    0xFF

// ACS724LMATR-50AU-T calibration (3.3 V supply, 12-bit ADC, unidirectional 0–50 A)
// VIOUT = 0.1×VCC + Sensitivity×I  →  at 0 A: 0.33 V → ADC raw ≈ 410
// Sensitivity ≈ 26.4 mV/A → ≈ 32 ADC counts per ampere
// Sensor range: 0–50 A → ADC 410–2048
#define ACS724_ADC_ZERO     410   // raw ADC at 0 A
#define ACS724_ADC_PER_AMP   32   // ADC counts per ampere

// Fixed operational constants (not user-tunable)
#define ADC_SAMPLES           8
#define FLASH_RX_MS           200UL
#define FLASH_TX_MS           200UL
#define FLASH_NET_MS          400UL

// NVS
#define NVS_NAMESPACE    "tanknode"
#define NVS_CFG_VER_KEY  "cfgver"
#define NVS_CFG_KEY      "cfg"
#define CONFIG_VERSION   3  // bumped: dryrun_grace_ticks added

// Defaults for NodeConfig (applied on first boot or after version mismatch)
#define DEF_PUMP_MIN_RUNTIME_MS        30000UL
#define DEF_PUMP_MIN_COOLDOWN_MS       60000UL
#define DEF_REPLENISH_RUNON_MS        300000UL
#define DEF_TELEMETRY_INTERVAL_MS      30000UL
#define DEF_NETWORK_TIMEOUT_MS         60000UL
#define DEF_ACK_TIMEOUT_MS            10000UL  // LoRa round-trip at 868/SF9 + margin
#define DEF_ACK_MAX_RETRIES                5
#define DEF_OVERCURRENT_THRESH  (ACS724_ADC_ZERO + 40 * ACS724_ADC_PER_AMP)  // ~40 A → raw 1690
#define DEF_DRYRUN_THRESH       (ACS724_ADC_ZERO +  2 * ACS724_ADC_PER_AMP)  //  ~2 A → raw  474
#define DEF_BOOT_AUTO_MODE                 1
#define DEF_OVERCURRENT_GRACE_TICKS        5   // 5 × 50 ms = 250 ms inrush window
#define DEF_DRYRUN_GRACE_TICKS            15   // 15 × 50 ms = 750 ms — air bubbles cause transient dips
#define DEF_FAULT_LOCKOUT_ENABLED          1   // 1=kill relays on fault  0=warn only

// Capacitive touch button polarity
// 1 = output LOW when touched (fell())  — most TTP223 modules with default pad
// 0 = output HIGH when touched (rose()) — TTP223 with A-pad bridged
#define CAP_BTN_ACTIVE_LOW  1

// =============================================================================
// SECTION 3 – PACKED NETWORK PROTOCOL STRUCTURES
// =============================================================================

enum MessageType : uint8_t {
    MSG_TELEMETRY   = 1,
    MSG_COMMAND     = 2,
    MSG_ACK         = 3,
    MSG_CONFIG_GET  = 4,  // gateway→node: request config (no payload)
    MSG_CONFIG_SET  = 5,  // gateway→node: write config   (NodeConfig payload)
    MSG_CONFIG_RESP = 6,  // node→gateway: current config (NodeConfig payload)
    MSG_STATS_GET   = 7,  // gateway→node: request stats  (no payload)
    MSG_STATS_RESP  = 8,  // node→gateway: operational stats (StatsPayload)
};

struct __attribute__((packed)) LoRaHeader {
    uint16_t magic_word;
    uint8_t  target_id;
    uint8_t  sender_id;
    uint8_t  msg_id;
    uint8_t  msg_type;
};  // 6 bytes

struct __attribute__((packed)) TelemetryData {
    uint8_t  water_level;   // 0–3  (3 float switches)
    uint8_t  system_flags;  // bit0=Auto bit1=P1 bit2=P2 bit3=P3 bit4=PondP
    int16_t  temperature;   // °C × 10
    uint8_t  humidity;      // % RH
    uint8_t  error_code;    // 0=OK 1=OC 2=DryRun 3=Comms
    int8_t   last_rssi;     // dBm of last received packet
    int8_t   last_snr;      // SNR in dB
};  // 8 bytes

struct __attribute__((packed)) CommandData {
    uint8_t target_pump;    // 1-4; doubled as acked-msg-id in ACK frames
    uint8_t action;         // 0=Off 1=On
    uint8_t flags;          // bit0=Force Override
};  // 3 bytes

// Runtime-configurable parameters — stored in NVS, transported over LoRa
struct __attribute__((packed)) NodeConfig {
    uint32_t pump_min_runtime_ms;    //  4   default 30 000
    uint32_t pump_min_cooldown_ms;   //  4   default 60 000
    uint32_t replenish_runon_ms;     //  4   default 300 000
    uint32_t telemetry_interval_ms;  //  4   default 30 000
    uint32_t network_timeout_ms;     //  4   default 60 000
    uint32_t ack_timeout_ms;         //  4   default 10 000
    uint16_t overcurrent_thresh;     //  2   default 3200 (12-bit ADC raw)
    uint16_t dryrun_thresh;          //  2   default 150
    uint8_t  ack_max_retries;          //  1   default 5
    uint8_t  boot_auto_mode;           //  1   1=auto 0=manual
    uint8_t  overcurrent_grace_ticks;  //  1   consecutive 50ms ticks before OC fault trips
    uint8_t  fault_lockout_enabled;    //  1   1=kill relays + lockout  0=warn only (no relay cut)
    uint8_t  dryrun_grace_ticks;       //  1   consecutive 50ms ticks before dry-run fault trips
};  // 33 bytes

// Operational statistics — persisted in NVS, queryable over LoRa
struct __attribute__((packed)) StatsPayload {
    uint32_t uptime_s;         //  4   seconds since last boot (not persisted)
    uint32_t runtime_p1_s;    //  4   accumulated P1 ON time
    uint32_t runtime_p2_s;    //  4
    uint32_t runtime_p3_s;    //  4
    uint32_t runtime_pond_s;  //  4
    uint16_t fill_cycles;      //  2   replenishment starts (leak indicator)
    uint16_t boot_count;       //  2   total reboots
    uint8_t  last_fault;       //  1   fault code at last fault event
    uint8_t  reserved[7];      //  7
};  // 32 bytes

struct __attribute__((packed)) LoRaPacket {
    LoRaHeader header;   // 6 bytes
    union {
        TelemetryData telemetry;   //  8
        CommandData   command;      //  3
        NodeConfig    config;       // 33
        StatsPayload  stats;        // 32
    } payload;           // 33 bytes (largest member)
};  // total 39 bytes

static_assert(sizeof(LoRaPacket)   <= 255, "LoRaPacket exceeds max RadioLib payload");
static_assert(sizeof(NodeConfig)   == 33,  "NodeConfig size mismatch");
static_assert(sizeof(StatsPayload) == 32,  "StatsPayload size mismatch");

// =============================================================================
// SECTION 4 – GLOBAL SYSTEM-STATE (mutex-protected)
// =============================================================================

struct SystemState {
    uint8_t  waterLevel;
    bool     autoMode;
    bool     relay_p1, relay_p2, relay_p3;
    bool     pondPump;
    uint8_t  errorCode;
    bool     faultLockout;
    bool     awaitingAck;
    uint32_t lastGatewayContact_ms;
    uint32_t lastPondContact_ms;
    float    temperature;
    float    humidity;
    bool     btnMode_edge, btnP1_edge, btnP2_edge, btnP3_edge, btnPond_edge;
    uint16_t currentP1, currentP2, currentP3;
    bool     rxFlash, txFlash;
    int8_t   lastRssi;   // updated after each valid RX by LoRaTransceiver
    int8_t   lastSnr;
};

static SystemState gState;

// =============================================================================
// SECTION 5 – FREERTOS SYNCHRONISATION HANDLES
// =============================================================================

static SemaphoreHandle_t xStateMutex       = nullptr;
static SemaphoreHandle_t xLoRaIrqSemaphore = nullptr;
static QueueHandle_t     xTxQueue          = nullptr;
static QueueHandle_t     xRxQueue          = nullptr;

// =============================================================================
// SECTION 6 – PERIPHERAL INSTANCES
// =============================================================================

static SPIClass          loraSPI(VSPI);
static SX1262            radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, loraSPI);
static CRGB              leds[NUM_LEDS];
static Adafruit_MCP23X17 mcp;

// =============================================================================
// SECTION 7 – REPLAY-ATTACK SEQUENCE TRACKING
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

// =============================================================================
// SECTION 8 – RADIOLIB DIO1 ISR
// =============================================================================

void IRAM_ATTR onPacketReceived() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xLoRaIrqSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// =============================================================================
// SECTION 9 – PERSISTENT CONFIG AND STATS
// =============================================================================

static NodeConfig    gConfig;
static StatsPayload  gStats;

static void resetConfigToDefaults() {
    gConfig.pump_min_runtime_ms   = DEF_PUMP_MIN_RUNTIME_MS;
    gConfig.pump_min_cooldown_ms  = DEF_PUMP_MIN_COOLDOWN_MS;
    gConfig.replenish_runon_ms    = DEF_REPLENISH_RUNON_MS;
    gConfig.telemetry_interval_ms = DEF_TELEMETRY_INTERVAL_MS;
    gConfig.network_timeout_ms    = DEF_NETWORK_TIMEOUT_MS;
    gConfig.ack_timeout_ms        = DEF_ACK_TIMEOUT_MS;
    gConfig.overcurrent_thresh    = DEF_OVERCURRENT_THRESH;
    gConfig.dryrun_thresh         = DEF_DRYRUN_THRESH;
    gConfig.ack_max_retries          = DEF_ACK_MAX_RETRIES;
    gConfig.boot_auto_mode           = DEF_BOOT_AUTO_MODE;
    gConfig.overcurrent_grace_ticks  = DEF_OVERCURRENT_GRACE_TICKS;
    gConfig.fault_lockout_enabled    = DEF_FAULT_LOCKOUT_ENABLED;
    gConfig.dryrun_grace_ticks       = DEF_DRYRUN_GRACE_TICKS;
}

// Sanity-check incoming config before applying it.
static bool validateConfig(const NodeConfig &c) {
    if (c.pump_min_runtime_ms   < 5000UL   || c.pump_min_runtime_ms   > 3600000UL) return false;
    if (c.pump_min_cooldown_ms  < 5000UL   || c.pump_min_cooldown_ms  > 3600000UL) return false;
    if (c.replenish_runon_ms    < 30000UL  || c.replenish_runon_ms    > 86400000UL) return false;
    if (c.telemetry_interval_ms < 5000UL   || c.telemetry_interval_ms > 3600000UL) return false;
    if (c.network_timeout_ms    < 10000UL  || c.network_timeout_ms    > 3600000UL) return false;
    if (c.ack_timeout_ms        < 1000UL   || c.ack_timeout_ms        > 60000UL)  return false;
    if (c.overcurrent_thresh    > 4095)                                            return false;
    if (c.dryrun_thresh         >= c.overcurrent_thresh)                           return false;
    if (c.ack_max_retries        < 1        || c.ack_max_retries       > 10)       return false;
    if (c.overcurrent_grace_ticks < 1)                                            return false;
    if (c.dryrun_grace_ticks < 1)                                                 return false;
    return true;
}

static void saveConfig() {
    Preferences p;
    p.begin(NVS_NAMESPACE, false);
    p.putUChar(NVS_CFG_VER_KEY, CONFIG_VERSION);
    p.putBytes(NVS_CFG_KEY, &gConfig, sizeof(gConfig));
    p.end();
}

// Saves all mutable stats fields except uptime_s and boot_count.
// boot_count is incremented and saved once in setup().
// uptime_s is ephemeral (set from millis() at query time).
static void saveStats() {
    Preferences p;
    p.begin(NVS_NAMESPACE, false);
    p.putUInt("rt1", gStats.runtime_p1_s);
    p.putUInt("rt2", gStats.runtime_p2_s);
    p.putUInt("rt3", gStats.runtime_p3_s);
    p.putUInt("rtp", gStats.runtime_pond_s);
    p.putUShort("fc",  gStats.fill_cycles);
    p.putUChar("lf",   gStats.last_fault);
    p.end();
}

static void loadConfig() {
    Preferences p;
    p.begin(NVS_NAMESPACE, true);  // read-only

    // Config blob
    uint8_t ver = p.getUChar(NVS_CFG_VER_KEY, 0);
    if (ver == CONFIG_VERSION && p.getBytesLength(NVS_CFG_KEY) == sizeof(gConfig)) {
        p.getBytes(NVS_CFG_KEY, &gConfig, sizeof(gConfig));
    } else {
        resetConfigToDefaults();
    }

    // Stats
    memset(&gStats, 0, sizeof(gStats));
    gStats.runtime_p1_s   = p.getUInt("rt1", 0);
    gStats.runtime_p2_s   = p.getUInt("rt2", 0);
    gStats.runtime_p3_s   = p.getUInt("rt3", 0);
    gStats.runtime_pond_s = p.getUInt("rtp", 0);
    gStats.fill_cycles     = p.getUShort("fc", 0);
    gStats.boot_count      = p.getUShort("bc", 0);
    gStats.last_fault      = p.getUChar("lf", 0);
    gStats.uptime_s        = 0;  // computed at query time
    p.end();
}

// =============================================================================
// SECTION 10 – SHARED HELPER FUNCTIONS
// =============================================================================

// Triangle-wave brightness oscillator: returns 0-255 with given period.
// Uses FastLED triwave8 — rises 0→255 over first half, falls back to 0.
// 'offset_ms' shifts the phase so multiple LEDs can breathe out of sync.
static inline uint8_t triWave(uint32_t now_ms, uint32_t period_ms, uint32_t offset_ms = 0) {
    return triwave8((uint8_t)(((now_ms + offset_ms) % period_ms) * 256UL / period_ms));
}

static uint16_t readCurrentADC(uint8_t pin) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < ADC_SAMPLES; i++) sum += (uint32_t)analogRead(pin);
    return (uint16_t)(sum / ADC_SAMPLES);
}

static void buildAndQueueTelemetry() {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.header.magic_word = MY_NETWORK_MAGIC;
    pkt.header.target_id  = NODE_GATEWAY;
    pkt.header.sender_id  = NODE_TANK_LOCAL;
    pkt.header.msg_id     = nextMsgId();
    pkt.header.msg_type   = MSG_TELEMETRY;

    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
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
    pkt.payload.telemetry.last_rssi    = gState.lastRssi;
    pkt.payload.telemetry.last_snr     = gState.lastSnr;
    xSemaphoreGive(xStateMutex);
    xQueueSend(xTxQueue, &pkt, 0);
}

static uint8_t sendPumpCommand(uint8_t target_node, uint8_t pump_id, uint8_t action) {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.header.magic_word        = MY_NETWORK_MAGIC;
    pkt.header.target_id         = target_node;
    pkt.header.sender_id         = NODE_TANK_LOCAL;
    pkt.header.msg_id            = nextMsgId();
    pkt.header.msg_type          = MSG_COMMAND;
    pkt.payload.command.target_pump = pump_id;
    pkt.payload.command.action      = action;
    pkt.payload.command.flags       = 0;
    xQueueSend(xTxQueue, &pkt, 0);
    return pkt.header.msg_id;
}

static void sendAck(uint8_t target_id, uint8_t acked_msg_id) {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.header.magic_word           = MY_NETWORK_MAGIC;
    pkt.header.target_id            = target_id;
    pkt.header.sender_id            = NODE_TANK_LOCAL;
    pkt.header.msg_id               = nextMsgId();
    pkt.header.msg_type             = MSG_ACK;
    pkt.payload.command.target_pump = acked_msg_id;
    pkt.payload.command.action      = 0;
    pkt.payload.command.flags       = 0;
    xQueueSend(xTxQueue, &pkt, 0);
}

// Sends the current gConfig as a CONFIG_RESP.  Gateway verifies applied values.
static void sendConfigResp(uint8_t target_id) {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.header.magic_word = MY_NETWORK_MAGIC;
    pkt.header.target_id  = target_id;
    pkt.header.sender_id  = NODE_TANK_LOCAL;
    pkt.header.msg_id     = nextMsgId();
    pkt.header.msg_type   = MSG_CONFIG_RESP;
    pkt.payload.config    = gConfig;
    xQueueSend(xTxQueue, &pkt, 0);
}

// Sends current operational statistics.  Called from ControlEngine only.
static void sendStatsResp(uint8_t target_id) {
    gStats.uptime_s = millis() / 1000;
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.header.magic_word = MY_NETWORK_MAGIC;
    pkt.header.target_id  = target_id;
    pkt.header.sender_id  = NODE_TANK_LOCAL;
    pkt.header.msg_id     = nextMsgId();
    pkt.header.msg_type   = MSG_STATS_RESP;
    pkt.payload.stats     = gStats;
    xQueueSend(xTxQueue, &pkt, 0);
}

// =============================================================================
// SECTION 11 – PUMP HYSTERESIS MANAGEMENT
// =============================================================================

struct PumpHysteresis {
    bool     state;
    uint32_t lastOnTime_ms;
    uint32_t lastOffTime_ms;
    bool     initialized;
};

// These read gConfig directly — only called from ControlEngine (no mutex needed).
static bool canTurnPumpOn(const PumpHysteresis &ph) {
    if (!ph.initialized) return true;
    if (ph.state)         return false;
    return (millis() - ph.lastOffTime_ms) >= gConfig.pump_min_cooldown_ms;
}

static bool canTurnPumpOff(const PumpHysteresis &ph) {
    if (!ph.initialized) return true;
    if (!ph.state)        return false;
    return (millis() - ph.lastOnTime_ms) >= gConfig.pump_min_runtime_ms;
}

static void setPumpState(PumpHysteresis &ph, bool newState, int relay_pin) {
    if (ph.initialized && (newState == ph.state)) return;
    if (relay_pin >= 0) digitalWrite((uint8_t)relay_pin, newState ? HIGH : LOW);
    if (newState) ph.lastOnTime_ms  = millis();
    else          ph.lastOffTime_ms = millis();
    ph.state       = newState;
    ph.initialized = true;
}

// Run-time tracking helpers – called from ControlEngine alongside setPumpState.
// onSince == 0 means pump was not tracked as running.
static inline void trackPumpOn(uint32_t &onSince) {
    if (onSince == 0) onSince = millis();
}

static inline uint32_t trackPumpOff(uint32_t &onSince) {
    if (onSince == 0) return 0;
    uint32_t elapsed = (millis() - onSince) / 1000UL;
    onSince = 0;
    return elapsed;
}

// =============================================================================
// SECTION 12 – TASK: INPUT SENSOR POLL  (Priority 3, Core 1, 20 ms)
// =============================================================================

static void Task_InputSensorPoll(void *pvParams) {
    const uint8_t curPins[3] = { CURRENT_P1_ADC, CURRENT_P2_ADC, CURRENT_P3_ADC };

    // Per-pin debounce state for all 8 MCP GPA pins.
    // Each entry tracks the last confirmed stable level, the candidate level
    // being debounced, and how many consecutive 20 ms ticks it has held.
    //
    // Buttons  (GPA0-4): cap module drives line; 1 tick  =  20 ms window
    // Floats   (GPA5-7): mechanical reed/float;  100 ticks = 2 000 ms window
    struct PinDb {
        bool    stable;   // last committed state (HIGH on reset with pull-up)
        bool    pending;  // candidate state currently being timed
        uint8_t count;    // consecutive ticks at 'pending'
        uint8_t thresh;   // ticks required to commit
    };

    PinDb db[8];
    for (uint8_t i = 0; i < 8; i++) {
        db[i] = { true, true, 0, (i < 5) ? (uint8_t)1 : (uint8_t)100 };
    }

    uint8_t mcpErrCount = 0;  // consecutive suspicious reads (I2C bus error guard)

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));

        // Read all 8 GPA pins in one I2C transaction
        uint8_t port = mcp.readGPIOA();

        // All 5 button lines simultaneously LOW is physically impossible with 5 independent
        // TTP223 modules. readGPIOA() returns 0x00 on I2C bus error — skip the tick to
        // avoid injecting garbage state (all buttons pressed + tank empty signal).
        if ((port & 0x1F) == 0x00) {
            if (++mcpErrCount >= 3) continue;  // 3 bad reads in a row → skip
        } else {
            mcpErrCount = 0;
        }

        bool fell[8] = {};   // HIGH→LOW transition confirmed this tick
        bool rose[8] = {};   // LOW→HIGH transition confirmed this tick

        for (uint8_t i = 0; i < 8; i++) {
            bool raw = (port >> i) & 1u;
            if (raw == db[i].pending) {
                if (db[i].count < db[i].thresh) db[i].count++;
                if (db[i].count >= db[i].thresh && raw != db[i].stable) {
                    fell[i]      = db[i].stable && !raw;
                    rose[i]      = !db[i].stable && raw;
                    db[i].stable = raw;
                }
            } else {
                // Direction changed before threshold — restart the timer
                db[i].pending = raw;
                db[i].count   = 0;
            }
        }

        // Cap-touch buttons: GPA0-4 — active low (TTP223 default) or active high
        bool btnEdge[5];
        for (uint8_t i = 0; i < 5; i++)
            btnEdge[i] = CAP_BTN_ACTIVE_LOW ? fell[i] : rose[i];

        // Float switches: GPA5-7 — active low (pull-up + switch to GND)
        // Level = highest consecutive LOW switch from bottom (0) upward.
        uint8_t level = 0;
        for (uint8_t i = 0; i < 3; i++) {
            if (!db[MCP_FLOAT_0 + i].stable) level = (uint8_t)(i + 1);
            else break;
        }

        // Current sensors (direct ESP32 ADC)
        uint16_t cur[3];
        for (uint8_t i = 0; i < 3; i++) cur[i] = readCurrentADC(curPins[i]);

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            gState.waterLevel = level;
            gState.currentP1  = cur[0];
            gState.currentP2  = cur[1];
            gState.currentP3  = cur[2];
            if (btnEdge[0]) gState.btnMode_edge = true;
            if (btnEdge[1]) gState.btnP1_edge   = true;
            if (btnEdge[2]) gState.btnP2_edge   = true;
            if (btnEdge[3]) gState.btnP3_edge   = true;
            if (btnEdge[4]) gState.btnPond_edge  = true;
            xSemaphoreGive(xStateMutex);
        }
    }
}

// =============================================================================
// SECTION 13 – TASK: CONTROL ENGINE  (Priority 4, Core 1, 50 ms)
// =============================================================================

static void Task_ControlEngine(void *pvParams) {
    PumpHysteresis ph_p1   = { false, 0, 0, false };
    PumpHysteresis ph_p2   = { false, 0, 0, false };
    PumpHysteresis ph_p3   = { false, 0, 0, false };
    PumpHysteresis ph_pond = { false, 0, 0, false };

    // Run-time tracking (seconds pump has been ON this session + NVS base)
    uint32_t onSince_p1   = 0;
    uint32_t onSince_p2   = 0;
    uint32_t onSince_p3   = 0;
    uint32_t onSince_pond = 0;

    uint8_t  ocGrace[3] = {0, 0, 0};   // consecutive overcurrent ticks per pump (P1/P2/P3)
    uint8_t  drGrace[3] = {0, 0, 0};   // consecutive dry-run ticks per pump

    bool     replenishActive    = false;
    uint32_t replenishStartTime = 0;

    bool     statsDirty       = false;   // true when gStats has unsaved changes
    uint32_t lastStatsSave_ms = 0;       // throttles NVS flash writes to ≤1 per 5 min

    bool     awaitingAckInternal = false;
    uint32_t ackSentTime_ms      = 0;
    uint8_t  ackRetryCount       = 0;
    uint8_t  pendingPondAction   = 0;

    uint32_t lastTelemetryTime_ms = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));

        uint32_t now = millis();

        // ── Snapshot global state ─────────────────────────────────────
        uint8_t  waterLevel, errorCode;
        bool     autoMode, faultLockout;
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
        btnMode = gState.btnMode_edge; gState.btnMode_edge = false;
        btnP1   = gState.btnP1_edge;   gState.btnP1_edge   = false;
        btnP2   = gState.btnP2_edge;   gState.btnP2_edge   = false;
        btnP3   = gState.btnP3_edge;   gState.btnP3_edge   = false;
        btnPond = gState.btnPond_edge;  gState.btnPond_edge  = false;

        xSemaphoreGive(xStateMutex);

        // ── Overcurrent / dry-run protection (inrush grace period) ──────────────────
        // Grace counters accumulate consecutive 50 ms ticks above/below threshold.
        // A fault only triggers after overcurrent_grace_ticks consecutive detections
        // (default 5 = 250 ms), filtering inrush spikes.  When fault_lockout_enabled=0
        // the relays keep running and errorCode is a warning only; it auto-clears.
        {
            const uint16_t curArr[3]  = { curP1, curP2, curP3 };
            const bool     runArr[3]  = { ph_p1.state, ph_p2.state, ph_p3.state };
            bool overCurrent = false, dryRun = false;

            for (uint8_t i = 0; i < 3; i++) {
                if (runArr[i]) {
                    if (curArr[i] > gConfig.overcurrent_thresh) {
                        if (ocGrace[i] < 255) ocGrace[i]++;
                    } else { ocGrace[i] = 0; }
                    if (curArr[i] < gConfig.dryrun_thresh) {
                        if (drGrace[i] < 255) drGrace[i]++;
                    } else { drGrace[i] = 0; }
                    if (ocGrace[i] >= gConfig.overcurrent_grace_ticks) overCurrent = true;
                    if (drGrace[i] >= gConfig.dryrun_grace_ticks)      dryRun      = true;
                } else {
                    ocGrace[i] = drGrace[i] = 0;
                }
            }

            if (overCurrent || dryRun) {
                uint8_t faultCode = overCurrent ? 1 : 2;
                if (gConfig.fault_lockout_enabled) {
                    // Hard trip – kill relays immediately, bypass hysteresis
                    digitalWrite(RELAY_P1, LOW); gStats.runtime_p1_s += trackPumpOff(onSince_p1);
                    digitalWrite(RELAY_P2, LOW); gStats.runtime_p2_s += trackPumpOff(onSince_p2);
                    digitalWrite(RELAY_P3, LOW); gStats.runtime_p3_s += trackPumpOff(onSince_p3);
                    ph_p1.state = false; ph_p1.lastOffTime_ms = now;
                    ph_p2.state = false; ph_p2.lastOffTime_ms = now;
                    ph_p3.state = false; ph_p3.lastOffTime_ms = now;
                    gStats.last_fault = faultCode;
                    saveStats();
                    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        gState.relay_p1 = false; gState.relay_p2 = false; gState.relay_p3 = false;
                        gState.faultLockout = true;
                        xSemaphoreGive(xStateMutex);
                    }
                }
                // Always set errorCode (warning or lockout)
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    gState.errorCode = faultCode;
                    xSemaphoreGive(xStateMutex);
                }
                buildAndQueueTelemetry();
                if (gConfig.fault_lockout_enabled) continue;  // halt remaining logic if locked out

            } else if (!faultLockout) {
                // Auto-clear warning once current returns to normal
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    if (gState.errorCode == 1 || gState.errorCode == 2) gState.errorCode = 0;
                    xSemaphoreGive(xStateMutex);
                }
            }
        }

        // ── Fault-lockout gate ──────────────────────────────────────────
        if (faultLockout) {
            if (btnMode) {
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    gState.faultLockout = false;
                    gState.errorCode    = 0;
                    xSemaphoreGive(xStateMutex);
                }
            }
            bool networkOkWhileFaulted =
                ((now - lastGateway) < gConfig.network_timeout_ms) &&
                ((now - lastPond)    < gConfig.network_timeout_ms);
            if (!networkOkWhileFaulted) {
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    if (gState.errorCode == 0) gState.errorCode = 3;
                    xSemaphoreGive(xStateMutex);
                }
            }
            continue;
        }

        // ── Network-timeout assessment ───────────────────────────────────
        bool networkOk = ((now - lastGateway) < gConfig.network_timeout_ms) &&
                         ((now - lastPond)    < gConfig.network_timeout_ms);

        // ── Mode toggle ────────────────────────────────────────────
        if (btnMode) {
            autoMode = !autoMode;
            if (!autoMode && replenishActive) replenishActive = false;
            gConfig.boot_auto_mode = autoMode ? 1 : 0;
            saveConfig();   // persist mode preference across reboots
            if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                gState.autoMode = autoMode;
                xSemaphoreGive(xStateMutex);
            }
        }

        // ── Process received LoRa packets ──────────────────────────────────
        LoRaPacket rxPkt;
        while (xQueueReceive(xRxQueue, &rxPkt, 0) == pdTRUE) {

            if (rxPkt.header.msg_type == MSG_COMMAND) {
                if (!isValidMsgId(rxPkt.header.sender_id, rxPkt.header.msg_id)) continue;
                CommandData &cmd  = rxPkt.payload.command;
                bool forceOverride = (cmd.flags & 0x01) != 0;

                if (autoMode && !forceOverride) {
                    sendAck(rxPkt.header.sender_id, rxPkt.header.msg_id);
                    continue;
                }

                switch (cmd.target_pump) {
                    case 1:
                        if (cmd.action == 1 && canTurnPumpOn(ph_p1)) {
                            setPumpState(ph_p1, true,  RELAY_P1);
                            trackPumpOn(onSince_p1);
                        } else if (cmd.action == 0 && canTurnPumpOff(ph_p1)) {
                            gStats.runtime_p1_s += trackPumpOff(onSince_p1);
                            setPumpState(ph_p1, false, RELAY_P1);
                            statsDirty = true;
                        }
                        break;
                    case 2:
                        if (cmd.action == 1 && canTurnPumpOn(ph_p2)) {
                            setPumpState(ph_p2, true,  RELAY_P2);
                            trackPumpOn(onSince_p2);
                        } else if (cmd.action == 0 && canTurnPumpOff(ph_p2)) {
                            gStats.runtime_p2_s += trackPumpOff(onSince_p2);
                            setPumpState(ph_p2, false, RELAY_P2);
                            statsDirty = true;
                        }
                        break;
                    case 3:
                        if (cmd.action == 1 && canTurnPumpOn(ph_p3)) {
                            setPumpState(ph_p3, true,  RELAY_P3);
                            trackPumpOn(onSince_p3);
                        } else if (cmd.action == 0 && canTurnPumpOff(ph_p3)) {
                            gStats.runtime_p3_s += trackPumpOff(onSince_p3);
                            setPumpState(ph_p3, false, RELAY_P3);
                            statsDirty = true;
                        }
                        break;
                    default: break;
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

            } else if (rxPkt.header.msg_type == MSG_CONFIG_GET) {
                if (!isValidMsgId(rxPkt.header.sender_id, rxPkt.header.msg_id)) continue;
                sendConfigResp(rxPkt.header.sender_id);

            } else if (rxPkt.header.msg_type == MSG_CONFIG_SET) {
                if (!isValidMsgId(rxPkt.header.sender_id, rxPkt.header.msg_id)) continue;
                if (validateConfig(rxPkt.payload.config)) {
                    gConfig = rxPkt.payload.config;
                    saveConfig();
                    // Reflect new auto-mode preference immediately
                    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        gState.autoMode = (gConfig.boot_auto_mode != 0);
                        autoMode        = gState.autoMode;
                        xSemaphoreGive(xStateMutex);
                    }
                }
                // Always respond with the current (possibly unchanged) config so
                // the gateway can detect if validation rejected the set.
                sendConfigResp(rxPkt.header.sender_id);

            } else if (rxPkt.header.msg_type == MSG_STATS_GET) {
                if (!isValidMsgId(rxPkt.header.sender_id, rxPkt.header.msg_id)) continue;
                sendStatsResp(rxPkt.header.sender_id);
            }
        }

        // ── ACK timeout / retry ──────────────────────────────────────────
        if (awaitingAckInternal && (now - ackSentTime_ms) >= gConfig.ack_timeout_ms) {
            if (ackRetryCount < gConfig.ack_max_retries) {
                ackRetryCount++;
                ackSentTime_ms = now;
                sendPumpCommand(NODE_POND_REMOTE, 1, pendingPondAction);
            } else {
                awaitingAckInternal = false;
                ackRetryCount       = 0;
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    gState.awaitingAck = false;
                    gState.errorCode   = 3;
                    xSemaphoreGive(xStateMutex);
                }
            }
        }

        // ── Network-timeout error field ──────────────────────────────────
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
                if (canTurnPumpOn(ph_pond)) {
                    replenishActive    = true;
                    replenishStartTime = now;
                    gStats.fill_cycles++;
                    statsDirty = true;  // fill_cycles is a leak-detection counter — flush soon
                    setPumpState(ph_pond, true, -1);
                    trackPumpOn(onSince_pond);
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
                bool runOnExpired = (now - replenishStartTime) >= gConfig.replenish_runon_ms;
                if (runOnExpired && waterLevel >= 3 && canTurnPumpOff(ph_pond)) {
                    replenishActive = false;
                    gStats.runtime_pond_s += trackPumpOff(onSince_pond);
                    setPumpState(ph_pond, false, -1);
                    statsDirty = true;
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
                if (want && canTurnPumpOn(ph_p1)) {
                    setPumpState(ph_p1, true, RELAY_P1);
                    trackPumpOn(onSince_p1);
                } else if (!want && canTurnPumpOff(ph_p1)) {
                    gStats.runtime_p1_s += trackPumpOff(onSince_p1);
                    setPumpState(ph_p1, false, RELAY_P1);
                    statsDirty = true;
                }
            }
            if (btnP2) {
                bool want = !ph_p2.state;
                if (want && canTurnPumpOn(ph_p2)) {
                    setPumpState(ph_p2, true, RELAY_P2);
                    trackPumpOn(onSince_p2);
                } else if (!want && canTurnPumpOff(ph_p2)) {
                    gStats.runtime_p2_s += trackPumpOff(onSince_p2);
                    setPumpState(ph_p2, false, RELAY_P2);
                    statsDirty = true;
                }
            }
            if (btnP3) {
                bool want = !ph_p3.state;
                if (want && canTurnPumpOn(ph_p3)) {
                    setPumpState(ph_p3, true, RELAY_P3);
                    trackPumpOn(onSince_p3);
                } else if (!want && canTurnPumpOff(ph_p3)) {
                    gStats.runtime_p3_s += trackPumpOff(onSince_p3);
                    setPumpState(ph_p3, false, RELAY_P3);
                    statsDirty = true;
                }
            }
            if (btnPond) {
                bool want = !ph_pond.state;
                if (want && canTurnPumpOn(ph_pond)) {
                    setPumpState(ph_pond, true, -1);
                    trackPumpOn(onSince_pond);
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
                    gStats.runtime_pond_s += trackPumpOff(onSince_pond);
                    setPumpState(ph_pond, false, -1);
                    statsDirty = true;
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

        // ── Push relay states ────────────────────────────────────────────
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            gState.relay_p1 = ph_p1.state;
            gState.relay_p2 = ph_p2.state;
            gState.relay_p3 = ph_p3.state;
            gState.pondPump = ph_pond.state;
            xSemaphoreGive(xStateMutex);
        }

        // ── Periodic NVS stats flush (max once per 5 min) ───────────────────
        if (statsDirty && (now - lastStatsSave_ms) >= 300000UL) {
            saveStats();
            statsDirty        = false;
            lastStatsSave_ms  = now;
        }

        // ── Periodic telemetry ───────────────────────────────────────────
        if ((now - lastTelemetryTime_ms) >= gConfig.telemetry_interval_ms) {
            lastTelemetryTime_ms = now;
            buildAndQueueTelemetry();
        }
    }
}

// =============================================================================
// SECTION 14 – TASK: LORA TRANSCEIVER  (Priority 2, Core 0, Event-driven)
// =============================================================================

static void Task_LoRaTransceiver(void *pvParams) {
    uint8_t rxBuf[sizeof(LoRaPacket)];

    for (;;) {
        // Drain TX queue first (non-blocking)
        LoRaPacket txPkt;
        if (xQueueReceive(xTxQueue, &txPkt, 0) == pdTRUE) {
            int state = radio.transmit((uint8_t *)&txPkt, (size_t)sizeof(LoRaPacket));
            if (state == RADIOLIB_ERR_NONE) {
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    gState.txFlash = true;
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
                            gState.rxFlash  = true;
                            gState.lastRssi = rssi;
                            gState.lastSnr  = snr;
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

// =============================================================================
// SECTION 15 – TASK: UI ANIMATION  (Priority 1, Core 0, 30 ms)
// =============================================================================

static void Task_UIAnimation(void *pvParams) {
    uint32_t lastRxFlash_ms = 0;
    uint32_t lastTxFlash_ms = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(30));
        uint32_t now = millis();

        // ── Snapshot state ───────────────────────────────────────────────
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

        fill_solid(leds, NUM_LEDS, CRGB::Black);

        // ── LEDs 0–2: water level ────────────────────────────────────────
        // Empty: slow red breathe on LED 0 (warning).
        // Filled: aqua column, brightness rises with level, top LED shimmers.
        // Filling (pond pump on): white sparkle chase climbs the column.
        if (waterLevel == 0) {
            uint8_t bri = scale8(triWave(now, 2000), 55);
            leds[0] = CRGB(bri, 0, 0);
        } else {
            for (uint8_t i = 0; i < waterLevel && i < 3; i++) {
                uint8_t base = (uint8_t)map(waterLevel, 1, 3, 100, 200);
                leds[i] = CRGB(0, (uint8_t)(base * 7 / 10), base);
            }
            // Top LED gets a gentle shimmer
            uint8_t sh  = scale8(triWave(now, 1800), 35);
            uint8_t top = waterLevel - 1;
            leds[top].g = qadd8(leds[top].g, sh);
            leds[top].b = qadd8(leds[top].b, sh / 2);
        }
        if (pondPump) {
            // Sparkle chases up LEDs 0→1→2 while filling
            uint8_t slot = (uint8_t)((now / 220) % 3);
            uint8_t bri  = scale8(triWave(now, 440), 210);
            leds[slot] = blend(leds[slot], CRGB(bri, bri, bri), 210);
        }

        // ── LED 3: heartbeat ───────────────────────────────────────────────
        // Lub-dub pulse every 1.6 s — confirms controller is alive.
        {
            uint32_t ph = now % 1600UL;
            uint8_t  hb = 0;
            if      (ph <  100) hb = (uint8_t)(ph * 2);
            else if (ph <  200) hb = (uint8_t)((200 - ph) * 2);
            else if (ph <  300) hb = (uint8_t)((ph - 200) * 2);
            else if (ph <  400) hb = (uint8_t)((400 - ph) * 2);
            hb = scale8(hb, 80);
            leds[3] = CRGB(hb, hb / 3, 0);   // warm amber
        }

        // ── LED 4: system status ─────────────────────────────────────────────
        if (faultLockout) {
            // Hard lockout: rapid red strobe ~8 Hz
            leds[4] = ((now / 62) & 1) ? CRGB(255, 0, 0) : CRGB::Black;

        } else if (errorCode == 1 || errorCode == 2) {
            // OC / dry-run warning: slow red breathe
            uint8_t bri = scale8(triWave(now, 900), 220) + 30;
            leds[4] = CRGB(bri, 0, 0);

        } else if (errorCode == 3) {
            // Network timeout: orange breathe
            uint8_t bri = scale8(triWave(now, 1200), 200) + 30;
            leds[4] = CRGB(bri, bri / 4, 0);

        } else if ((now - lastRxFlash_ms) < FLASH_RX_MS) {
            // RX received: purple flash that fades out
            uint8_t fade = (uint8_t)(255 - (now - lastRxFlash_ms) * 255UL / FLASH_RX_MS);
            leds[4] = CRGB(scale8(148, fade), 0, scale8(211, fade));

        } else if ((now - lastTxFlash_ms) < FLASH_TX_MS) {
            // TX sent: white flash that fades out
            uint8_t fade = (uint8_t)(255 - (now - lastTxFlash_ms) * 255UL / FLASH_TX_MS);
            leds[4] = CRGB(fade, fade, fade);

        } else if (awaitingAck) {
            // Waiting for pond ACK: fast yellow pulse
            uint8_t bri = scale8(triWave(now, 400), 230) + 20;
            leds[4] = CRGB(bri, bri, 0);

        } else if (autoMode) {
            // Auto idle: slow breathing green
            uint8_t bri = scale8(triWave(now, 2500), 190) + 40;
            leds[4] = CRGB(0, bri, 0);

        } else {
            // Manual idle: slow breathing blue
            uint8_t bri = scale8(triWave(now, 2500), 180) + 40;
            leds[4] = CRGB(0, 0, bri);
        }

        // ── LED 5: mode indicator ──────────────────────────────────────────
        // Breathes in complementary phase to LED 4; red when faulted.
        if (faultLockout || errorCode == 1 || errorCode == 2) {
            uint8_t bri = scale8(triWave(now, 900, 450), 140);
            leds[5] = CRGB(bri, 0, 0);
        } else if (autoMode) {
            uint8_t bri = scale8(triWave(now, 2500, 1250), 160) + 30;
            leds[5] = CRGB(0, bri, 0);
        } else {
            uint8_t bri = scale8(triWave(now, 2500, 1250), 150) + 30;
            leds[5] = CRGB(0, 0, bri);
        }

        // ── LEDs 6–8: local pump relays P1 / P2 / P3 ────────────────────────
        // ON  → bright green with occasional white sparkle.
        // OFF → very dim blue-grey idle breathe, staggered phases.
        const bool     relayOn[3]    = { relay_p1, relay_p2, relay_p3 };
        const uint32_t idleOff[3]    = { 0, 1100, 2200 };
        for (uint8_t i = 0; i < 3; i++) {
            if (relayOn[i]) {
                uint8_t sp = (random8() > 250) ? random8(50, 150) : 0;
                leds[6 + i] = CRGB(sp, qadd8(195, sp / 3), sp);
            } else {
                uint8_t bri = scale8(triWave(now, 3500, idleOff[i]), 16);
                leds[6 + i] = CRGB(bri / 4, bri / 4, bri);
            }
        }

        // ── LED 9: remote pond pump ──────────────────────────────────────────
        // ON  → bright cyan with slow shimmer.
        // OFF → dim teal idle breathe.
        if (pondPump) {
            uint8_t sh = scale8(triWave(now, 900), 50);
            leds[9] = CRGB(0, qadd8(150, sh), qadd8(180, sh));
        } else {
            uint8_t bri = scale8(triWave(now, 4000, 700), 20);
            leds[9] = CRGB(0, bri / 2, bri);
        }

        FastLED.show();
    }
}

// =============================================================================
// SECTION 16 – SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("\n[BOOT] Tank Controller Node 0x02 v1.1 starting...");

    // ── Load NVS config and stats ─────────────────────────────────────────────
    loadConfig();
    gStats.boot_count++;
    {
        Preferences p;
        p.begin(NVS_NAMESPACE, false);
        p.putUShort("bc", gStats.boot_count);
        p.end();
    }
    Serial.printf("[BOOT] Boot #%u  |  auto=%u  |  fill cycles=%u\n",
                  gStats.boot_count, gConfig.boot_auto_mode, gStats.fill_cycles);

    // ── I2C + MCP23017 ────────────────────────────────────────────────
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!mcp.begin_I2C(MCP_I2C_ADDR)) {
        Serial.println("[FATAL] MCP23017 not found at 0x20");
        fill_solid(leds, NUM_LEDS, CRGB::Red); FastLED.show();
        for (;;) vTaskDelay(portMAX_DELAY);
    }
    // Cap-touch buttons: TTP223 drives the line actively — no pull-up
    for (uint8_t i = MCP_BTN_MODE; i <= MCP_BTN_POND; i++) mcp.pinMode(i, INPUT);
    // Float switches: NC + switch to GND — enable 100 kΩ internal pull-up
    for (uint8_t i = MCP_FLOAT_0; i <= MCP_FLOAT_2; i++) mcp.pinMode(i, INPUT_PULLUP);

    // ── GPIO ─────────────────────────────────────────────────────────────────
    pinMode(RELAY_P1, OUTPUT); digitalWrite(RELAY_P1, LOW);
    pinMode(RELAY_P2, OUTPUT); digitalWrite(RELAY_P2, LOW);
    pinMode(RELAY_P3, OUTPUT); digitalWrite(RELAY_P3, LOW);

    analogSetWidth(12);
    analogSetAttenuation(ADC_11db);

    // ── FastLED ──────────────────────────────────────────────────────────────────
    FastLED.addLeds<WS2812B, WS2812B_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(80);
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();

    // ── Global state ──────────────────────────────────────────────────────────
    memset(&gState, 0, sizeof(gState));
    gState.autoMode              = (gConfig.boot_auto_mode != 0);
    gState.lastGatewayContact_ms = millis();
    gState.lastPondContact_ms    = millis();

    // ── FreeRTOS primitives ───────────────────────────────────────────────────
    xStateMutex = xSemaphoreCreateMutex();
    configASSERT(xStateMutex != nullptr);

    xLoRaIrqSemaphore = xSemaphoreCreateBinary();
    configASSERT(xLoRaIrqSemaphore != nullptr);

    xTxQueue = xQueueCreate(8, sizeof(LoRaPacket));
    configASSERT(xTxQueue != nullptr);

    xRxQueue = xQueueCreate(8, sizeof(LoRaPacket));
    configASSERT(xRxQueue != nullptr);

    // ── RadioLib SX1262 ─────────────────────────────────────────────────────
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    int radioState = radio.begin(
        LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SF,
        LORA_CR, LORA_SYNC_WORD, LORA_TX_POWER, LORA_PREAMBLE);

    if (radioState != RADIOLIB_ERR_NONE) {
        Serial.printf("[FATAL] RadioLib begin() failed: %d\n", radioState);
        fill_solid(leds, NUM_LEDS, CRGB::Red); FastLED.show();
        for (;;) vTaskDelay(portMAX_DELAY);
    }

    radio.setCRC(true);
    radio.setPacketReceivedAction(onPacketReceived);

    radioState = radio.startReceive();
    if (radioState != RADIOLIB_ERR_NONE) {
        Serial.printf("[FATAL] RadioLib startReceive() failed: %d\n", radioState);
        fill_solid(leds, NUM_LEDS, CRGB::Red); FastLED.show();
        for (;;) vTaskDelay(portMAX_DELAY);
    }

    Serial.printf("[BOOT] Radio OK  |  Packet size: %u bytes\n",
                  (unsigned)sizeof(LoRaPacket));

    // ── FreeRTOS tasks ─────────────────────────────────────────────────────────
    BaseType_t rc;

    // Priority scheme: pump control is most critical, LoRa is secondary.
    // Core 1: InputSensorPoll P3 feeds ControlEngine P4 (pump logic is king)
    // Core 0: LoRaTransceiver P2, UIAnimation P1
    rc = xTaskCreatePinnedToCore(Task_InputSensorPoll, "InputPoll", 4096, nullptr, 3, nullptr, 1);
    configASSERT(rc == pdPASS);

    rc = xTaskCreatePinnedToCore(Task_ControlEngine,   "CtrlEng",  8192, nullptr, 4, nullptr, 1);
    configASSERT(rc == pdPASS);

    rc = xTaskCreatePinnedToCore(Task_LoRaTransceiver, "LoRaTx",   8192, nullptr, 2, nullptr, 0);
    configASSERT(rc == pdPASS);

    rc = xTaskCreatePinnedToCore(Task_UIAnimation,     "UIAnim",   4096, nullptr, 1, nullptr, 0);
    configASSERT(rc == pdPASS);

    Serial.println("[BOOT] All tasks created.  System operational.");
}

// =============================================================================
// SECTION 17 – LOOP (all work is in RTOS tasks)
// =============================================================================

void loop() { vTaskDelay(portMAX_DELAY); }

// =============================================================================
// TANK CONTROLLER NODE (0x02)  –  Production Firmware v1.1
// Target  : Espressif ESP32 (Xtensa LX6 Dual-Core)
// Framework: Arduino + FreeRTOS
// Libraries: RadioLib (SX1262), FastLED (WS2812B), Wire/PCF8574, Preferences
//
// Protocol v1.1 vs v1.0:
//   • TelemetryData gains last_rssi (int8) and last_snr (int8) — wire-breaking change
//   • MSG_CONFIG_GET/SET/RESP  (types 4/5/6) — runtime config over LoRa
//   • MSG_STATS_GET/RESP       (types 7/8)   — pump run-times, fill cycles, boot info
//   • All timing/threshold constants now live in NVS-backed NodeConfig (gConfig)
//
// HARDWARE NOTES:
//   • GPIO 34-39: input-only, no internal pull-up.  Install 10 kΩ to 3.3 V.
//   • GPIO 2  (I2C_SCL): used for PCF8574. CURRENT_P3_ADC moved to GPIO 3 on S3.
//   • Relays P1/P2/P3: GPIO 47/41/39 (active-low). Buzzer: GPIO 21 (active-low).
//   • WS2812B: GPIO 40. LoRa MISO: GPIO 13 (S3), 19 (ESP32). BUSY: GPIO 17 (S3), 27 (ESP32).
//   • RadioLib transmit() blocks; at SF9/125 kHz max payload ≈ 330 ms.
//   • DHT22 removed — temperature/humidity fields in telemetry will be zero.
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <FastLED.h>
#include <Wire.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_system.h>
#include <stdint.h>
#include <string.h>

// =============================================================================
// SECTION 1 – PIN DEFINITIONS
// =============================================================================

#define LORA_NSS   5
#ifdef CONFIG_IDF_TARGET_ESP32S3
  #define LORA_DIO1  16
  #define LORA_BUSY  17
  #define LORA_MOSI  11
  #define LORA_MISO  13
#else
  #define LORA_DIO1  26
  #define LORA_BUSY  27
  #define LORA_MOSI  23
  #define LORA_MISO  19
#endif
#define LORA_NRST  14
#define LORA_SCK   18

#define WS2812B_PIN  48
#define NUM_LEDS     10

// I2C bus for PCF8574 I/O expander (float switches + buttons)
#define I2C_SDA  1
#define I2C_SCL  2

// PCF8574 address and bit assignments (8-bit, all inputs, active-low with pull-ups)
#define PCF8574_ADDR  0x27
#define PCF_FLOAT_0   0
#define PCF_FLOAT_1   1
#define PCF_FLOAT_2   2
#define PCF_BTN_MODE  3
#define PCF_BTN_P1    4
#define PCF_BTN_P2    5
#define PCF_BTN_P3    6
#define PCF_BTN_POND  7

// Relay and buzzer outputs (native GPIO, active-low: LOW=ON, HIGH=OFF)
#define RELAY_P1    47
#define RELAY_P2    41
#define RELAY_P3    39
#define BUZZER_PIN  21

#define CURRENT_P1_ADC  4   // ADC1_CH3 on S3 / ADC2_CH0 on ESP32
// GPIO 39 is not ADC-capable on ESP32-S3; ADC1 is GPIO 1-10, ADC2 is GPIO 11-20
#ifdef CONFIG_IDF_TARGET_ESP32S3
  #define CURRENT_P2_ADC   7  // ADC1_CH6 on S3
#else
  #define CURRENT_P2_ADC  39  // ADC1_CH3 on classic ESP32 (input-only, safe)
#endif
#ifdef CONFIG_IDF_TARGET_ESP32S3
  #define CURRENT_P3_ADC  3  // ADC1_CH2 on S3 (GPIO 2 reserved for I2C_SCL)
#else
  #define CURRENT_P3_ADC  2  // ADC2_CH2 on classic ESP32
#endif

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

// Fixed operational constants (not user-tunable)
#define ADC_SAMPLES           8
#define FLASH_RX_MS           200UL
#define FLASH_TX_MS           200UL
#define FLASH_NET_MS          400UL

// NVS
#define NVS_NAMESPACE    "tanknode"
#define NVS_CFG_VER_KEY  "cfgver"
#define NVS_CFG_KEY      "cfg"
#define CONFIG_VERSION   2  // bumped: overcurrent_grace_ticks + fault_lockout_enabled added

// Consecutive abnormal-reset counter stored in NVS.
// If radio init crashes (INT_WDT/Panic) repeatedly we skip init to break the loop.
#define NVS_RADIO_STREAK_KEY  "radio_cs"
#define RADIO_SKIP_STREAK     3   // skip radio after this many consecutive crash-boots during init

// Defaults for NodeConfig (applied on first boot or after version mismatch)
#define DEF_PUMP_MIN_RUNTIME_MS        30000UL
#define DEF_PUMP_MIN_COOLDOWN_MS       60000UL
#define DEF_REPLENISH_RUNON_MS        300000UL
#define DEF_TELEMETRY_INTERVAL_MS      30000UL
#define DEF_NETWORK_TIMEOUT_MS         60000UL
#define DEF_ACK_TIMEOUT_MS            10000UL  // LoRa round-trip at 868/SF9 + margin
#define DEF_ACK_MAX_RETRIES                5
#define DEF_OVERCURRENT_THRESH          3200
#define DEF_DRYRUN_THRESH                150
#define DEF_BOOT_AUTO_MODE                 1
#define DEF_OVERCURRENT_GRACE_TICKS        5   // 5 × 50 ms = 250 ms inrush window
#define DEF_FAULT_LOCKOUT_ENABLED          1   // 1=kill relays on fault  0=warn only

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
    uint8_t  overcurrent_grace_ticks;  //  1   consecutive 50ms ticks before fault trips (0=immediate)
    uint8_t  fault_lockout_enabled;    //  1   1=kill relays + lockout  0=warn only (no relay cut)
};  // 32 bytes

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
        NodeConfig    config;       // 32
        StatsPayload  stats;        // 32
    } payload;           // 32 bytes (largest member)
};  // total 38 bytes

static_assert(sizeof(LoRaPacket)   <= 255, "LoRaPacket exceeds max RadioLib payload");
static_assert(sizeof(NodeConfig)   == 32,  "NodeConfig size mismatch");
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
    uint32_t rxFlashTime_ms, txFlashTime_ms;
    int8_t   lastRssi;   // updated after each valid RX by LoRaTransceiver
    int8_t   lastSnr;
};

static SystemState gState;
static bool        gRadioOk = false;  // set true only after successful radio.begin()+startReceive()
static bool        gPcfOk   = false;  // set true only after PCF8574 ACKs its address in setup()

// =============================================================================
// SECTION 5 – FREERTOS SYNCHRONISATION HANDLES
// =============================================================================

static SemaphoreHandle_t xStateMutex       = nullptr;
static SemaphoreHandle_t xLoRaIrqSemaphore = nullptr;
static SemaphoreHandle_t xI2cMutex         = nullptr;  // guards Wire bus in Task_InputSensorPoll
static QueueHandle_t     xTxQueue          = nullptr;
static QueueHandle_t     xRxQueue          = nullptr;

// =============================================================================
// SECTION 6 – PERIPHERAL INSTANCES
// =============================================================================

#ifdef CONFIG_IDF_TARGET_ESP32S3
static SPIClass loraSPI(FSPI);   // SPI2 on ESP32-S3
#else
static SPIClass loraSPI(VSPI);   // VSPI on classic ESP32
#endif
static SX1262   radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, loraSPI);
static CRGB     leds[NUM_LEDS];

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
static uint8_t       gRadioCrashStreak = 0;  // consecutive crash-boots during radio init

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
    if (c.ack_max_retries       < 1        || c.ack_max_retries       > 10)       return false;
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
    gRadioCrashStreak = p.getUChar(NVS_RADIO_STREAK_KEY, 0);
    p.end();
}

// =============================================================================
// SECTION 10 – SHARED HELPER FUNCTIONS
// =============================================================================

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
        pkt.payload.telemetry.last_rssi    = gState.lastRssi;
        pkt.payload.telemetry.last_snr     = gState.lastSnr;
        xSemaphoreGive(xStateMutex);
    }
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

// relay_pin: GPIO for relay output (active-low); -1 = no local relay (pond, remote only)
static void setPumpState(PumpHysteresis &ph, bool newState, int8_t relay_pin) {
    if (ph.initialized && (newState == ph.state)) return;
    if (relay_pin >= 0) {
        digitalWrite((uint8_t)relay_pin, newState ? LOW : HIGH);  // active-low
    }
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
    uint8_t  prevBits        = 0xFF;
    uint32_t pcfRetryAt_ms   = 0;  // millis() timestamp for next PCF re-init attempt

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));

        // ── PCF8574 hot-plug recovery (retry every 5 s while not present) ────
        if (!gPcfOk && (millis() >= pcfRetryAt_ms)) {
            pcfRetryAt_ms = millis() + 5000;
            if (xSemaphoreTake(xI2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                Wire.beginTransmission(PCF8574_ADDR);
                Wire.write(0xFF);  // all inputs with pull-ups
                bool ok = (Wire.endTransmission() == 0);
                xSemaphoreGive(xI2cMutex);
                if (ok) {
                    gPcfOk = true;
                    prevBits = 0xFF;
                    Serial.println("\n[PCF] PCF8574 detected — float switch and button inputs enabled");
                }
            }
        }

        // Read all 8 PCF8574 pins in one I2C transaction (skipped if not present)
        uint8_t currBits = 0xFF;  // safe default: all HIGH → no water, no button presses
        if (gPcfOk && xSemaphoreTake(xI2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            Wire.requestFrom((uint8_t)PCF8574_ADDR, (uint8_t)1);
            if (Wire.available() >= 1) {
                currBits = (uint8_t)Wire.read();
            }
            xSemaphoreGive(xI2cMutex);
        }

        // Falling-edge mask: bits that transitioned HIGH→LOW this tick (active-low press)
        uint8_t fell = prevBits & ~currBits;
        prevBits = currBits;

        // Water level: count consecutive LOW float bits from bit 0 upward
        uint8_t level = 0;
        for (uint8_t i = 0; i < 3; i++) {
            if ((currBits & (1u << i)) == 0) level = (uint8_t)(i + 1);
            else break;
        }

        // ADC current sensing (native analog pins, unchanged)
        uint16_t cur[3];
        for (uint8_t i = 0; i < 3; i++) cur[i] = readCurrentADC(curPins[i]);

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            gState.waterLevel = level;
            gState.currentP1  = cur[0];
            gState.currentP2  = cur[1];
            gState.currentP3  = cur[2];
            if (fell & (1u << PCF_BTN_MODE)) gState.btnMode_edge = true;
            if (fell & (1u << PCF_BTN_P1))   gState.btnP1_edge   = true;
            if (fell & (1u << PCF_BTN_P2))   gState.btnP2_edge   = true;
            if (fell & (1u << PCF_BTN_P3))   gState.btnP3_edge   = true;
            if (fell & (1u << PCF_BTN_POND))  gState.btnPond_edge  = true;
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

    bool     awaitingAckInternal = false;
    uint32_t ackSentTime_ms      = 0;
    uint8_t  ackRetryCount       = 0;
    uint8_t  pendingPondAction   = 0;

    uint32_t lastTelemetryTime_ms = 0;
    uint32_t lastStatusPrint_ms   = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));

        uint32_t now = millis();

        // ── Snapshot global state ───────────────────────────────────────────
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

        // ── Live console status (overwrites same line every 2 s) ─────────────
        if ((now - lastStatusPrint_ms) >= 2000) {
            lastStatusPrint_ms = now;
            const char *errStr =
                faultLockout   ? "FAULT-LOCKOUT" :
                errorCode == 1 ? "OVERCURRENT"   :
                errorCode == 2 ? "DRY-RUN"       :
                errorCode == 3 ? "NO-COMMS"      : "OK";
            bool buzzerOn = (digitalRead(BUZZER_PIN) == LOW);
            Serial.printf("\r[%6lus] WL:%u %s | P1:%s P2:%s P3:%s Pond:%s | ADC:%4u/%4u/%4u | %s%s     ",
                now / 1000,
                waterLevel,
                autoMode ? "AUTO" : "MAN ",
                ph_p1.state   ? "ON " : "off",
                ph_p2.state   ? "ON " : "off",
                ph_p3.state   ? "ON " : "off",
                ph_pond.state ? "ON " : "off",
                curP1, curP2, curP3,
                errStr,
                buzzerOn ? " BUZZ" : "");
        }

        // ── Overcurrent / dry-run protection (inrush grace period) ───────────────
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
                    if (drGrace[i] >= gConfig.overcurrent_grace_ticks) dryRun      = true;
                } else {
                    ocGrace[i] = drGrace[i] = 0;
                }
            }

            if (overCurrent || dryRun) {
                uint8_t faultCode = overCurrent ? 1 : 2;
                if (gConfig.fault_lockout_enabled) {
                    // Hard trip – kill all relays and sound buzzer
                    digitalWrite(RELAY_P1, HIGH);
                    digitalWrite(RELAY_P2, HIGH);
                    digitalWrite(RELAY_P3, HIGH);
                    digitalWrite(BUZZER_PIN, LOW);
                    gStats.runtime_p1_s += trackPumpOff(onSince_p1);
                    gStats.runtime_p2_s += trackPumpOff(onSince_p2);
                    gStats.runtime_p3_s += trackPumpOff(onSince_p3);
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

        // ── Fault-lockout gate ────────────────────────────────────────────────
        if (faultLockout) {
            if (btnMode) {
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    gState.faultLockout = false;
                    gState.errorCode    = 0;
                    xSemaphoreGive(xStateMutex);
                }
                digitalWrite(BUZZER_PIN, HIGH);
            }
            bool networkOkWhileFaulted = !gRadioOk ||
                (((now - lastGateway) < gConfig.network_timeout_ms) &&
                 ((now - lastPond)    < gConfig.network_timeout_ms));
            if (!networkOkWhileFaulted) {
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    if (gState.errorCode == 0) gState.errorCode = 3;
                    xSemaphoreGive(xStateMutex);
                }
            }
            continue;
        }

        // ── Network-timeout assessment ───────────────────────────────────────
        // When radio is absent, treat network as always OK to suppress error code 3.
        bool networkOk = !gRadioOk ||
            (((now - lastGateway) < gConfig.network_timeout_ms) &&
             ((now - lastPond)    < gConfig.network_timeout_ms));

        // ── Mode toggle ──────────────────────────────────────────────────────
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

        // ── Process received LoRa packets ────────────────────────────────────
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
                            saveStats();
                        }
                        break;
                    case 2:
                        if (cmd.action == 1 && canTurnPumpOn(ph_p2)) {
                            setPumpState(ph_p2, true,  RELAY_P2);
                            trackPumpOn(onSince_p2);
                        } else if (cmd.action == 0 && canTurnPumpOff(ph_p2)) {
                            gStats.runtime_p2_s += trackPumpOff(onSince_p2);
                            setPumpState(ph_p2, false, RELAY_P2);
                            saveStats();
                        }
                        break;
                    case 3:
                        if (cmd.action == 1 && canTurnPumpOn(ph_p3)) {
                            setPumpState(ph_p3, true,  RELAY_P3);
                            trackPumpOn(onSince_p3);
                        } else if (cmd.action == 0 && canTurnPumpOff(ph_p3)) {
                            gStats.runtime_p3_s += trackPumpOff(onSince_p3);
                            setPumpState(ph_p3, false, RELAY_P3);
                            saveStats();
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

        // ── ACK timeout / retry ──────────────────────────────────────────────
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

        // ── Network-timeout error field ──────────────────────────────────────
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
                    saveStats();  // record fill cycle start; leak-detection counter
                    setPumpState(ph_pond, true, -1);
                    trackPumpOn(onSince_pond);
                    if (gRadioOk) {
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
                }
            } else if (replenishActive) {
                bool runOnExpired = (now - replenishStartTime) >= gConfig.replenish_runon_ms;
                if (runOnExpired && waterLevel >= 3 && canTurnPumpOff(ph_pond)) {
                    replenishActive = false;
                    gStats.runtime_pond_s += trackPumpOff(onSince_pond);
                    setPumpState(ph_pond, false, -1);
                    saveStats();
                    if (gRadioOk) {
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
                    saveStats();
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
                    saveStats();
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
                    saveStats();
                }
            }
            if (btnPond && gRadioOk) {
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
                    saveStats();
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

        // ── Push relay states ────────────────────────────────────────────────
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            gState.relay_p1 = ph_p1.state;
            gState.relay_p2 = ph_p2.state;
            gState.relay_p3 = ph_p3.state;
            gState.pondPump = ph_pond.state;
            xSemaphoreGive(xStateMutex);
        }

        // ── Periodic telemetry ───────────────────────────────────────────────
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

// =============================================================================
// SECTION 15 – TASK: UI ANIMATION  (Priority 1, Core 0, 30 ms)
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

// =============================================================================
// SECTION 16 – SETUP
// =============================================================================

static const char* resetReasonStr(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "Power-on";
        case ESP_RST_EXT:       return "External reset pin";
        case ESP_RST_SW:        return "Software reset";
        case ESP_RST_PANIC:     return "Panic / exception";
        case ESP_RST_INT_WDT:   return "Interrupt watchdog";
        case ESP_RST_TASK_WDT:  return "Task watchdog  ← likely a hang during init";
        case ESP_RST_WDT:       return "Other watchdog";
        case ESP_RST_BROWNOUT:  return "Brownout (low supply voltage)";
        case ESP_RST_DEEPSLEEP: return "Deep-sleep wakeup";
        default:                return "Unknown";
    }
}

void setup() {
    Serial.begin(115200);
    delay(3000);  // allow time for serial monitor to connect

    esp_reset_reason_t rr = esp_reset_reason();
    Serial.println("\n============================================================");
    Serial.println("[BOOT] Tank Controller Node 0x02 v1.1");
    Serial.printf( "[BOOT] Reset reason : %s\n", resetReasonStr(rr));
    if (rr == ESP_RST_TASK_WDT || rr == ESP_RST_INT_WDT || rr == ESP_RST_PANIC) {
        Serial.println("[BOOT] WARNING       : Abnormal reset — probably a hang or crash");
        Serial.println("[BOOT]                 during the previous boot's hardware init.");
        Serial.println("[BOOT]                 If this repeats, check SX1262 wiring.");
    }
    Serial.println("------------------------------------------------------------");

    // ── Load NVS config and stats ─────────────────────────────────────────────
    Serial.println("[INIT] Loading NVS config and stats...");
    loadConfig();
    gStats.boot_count++;
    {
        Preferences p;
        p.begin(NVS_NAMESPACE, false);
        p.putUShort("bc", gStats.boot_count);
        p.end();
    }
    Serial.printf("[BOOT] Boot #%u  |  auto=%u  |  fill_cycles=%u  |  last_fault=%u\n",
                  gStats.boot_count, gConfig.boot_auto_mode,
                  gStats.fill_cycles, gStats.last_fault);
    Serial.printf("[BOOT] Runtime  : P1=%lus  P2=%lus  P3=%lus  Pond=%lus\n",
                  gStats.runtime_p1_s, gStats.runtime_p2_s,
                  gStats.runtime_p3_s, gStats.runtime_pond_s);
    Serial.printf("[BOOT] Config   : min_run=%lus  cooldown=%lus  replenish=%lus\n",
                  gConfig.pump_min_runtime_ms / 1000,
                  gConfig.pump_min_cooldown_ms / 1000,
                  gConfig.replenish_runon_ms / 1000);
    Serial.printf("[BOOT] Radio crash streak: %u  (skip after %u)\n",
                  gRadioCrashStreak, (uint8_t)RADIO_SKIP_STREAK);
    Serial.println("------------------------------------------------------------");

    // ── GPIO ─────────────────────────────────────────────────────────────────
    // Relay outputs (active-low: HIGH = relay OFF)
    pinMode(RELAY_P1, OUTPUT); digitalWrite(RELAY_P1, HIGH);
    pinMode(RELAY_P2, OUTPUT); digitalWrite(RELAY_P2, HIGH);
    pinMode(RELAY_P3, OUTPUT); digitalWrite(RELAY_P3, HIGH);
    // Buzzer output (active-low: HIGH = silent)
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, HIGH);
    Serial.printf("[INIT] Relays: P1=GPIO%d  P2=GPIO%d  P3=GPIO%d  Buzzer=GPIO%d\n",
                  RELAY_P1, RELAY_P2, RELAY_P3, BUZZER_PIN);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    Serial.printf("[INIT] ADC pins: currentP1=GPIO%d  currentP2=GPIO%d  currentP3=GPIO%d\n",
                  CURRENT_P1_ADC, CURRENT_P2_ADC, CURRENT_P3_ADC);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.beginTransmission(PCF8574_ADDR);
    Wire.write(0xFF);  // all 8 pins: inputs with pull-ups enabled
    gPcfOk = (Wire.endTransmission() == 0);
    Serial.printf("[INIT] PCF8574 I2C: SDA=GPIO%d  SCL=GPIO%d  addr=0x%02X  %s\n",
                  I2C_SDA, I2C_SCL, PCF8574_ADDR,
                  gPcfOk ? "OK — float switches on bits 0-2, buttons on bits 3-7"
                         : "not found — float switch and button inputs disabled");

    // ── FastLED ───────────────────────────────────────────────────────────────
    Serial.printf("[INIT] WS2812B: GPIO%d  %d LEDs\n", WS2812B_PIN, NUM_LEDS);
    FastLED.addLeds<WS2812B, WS2812B_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(80);
    fill_solid(leds, NUM_LEDS, CRGB::Black);

    // ── Global state ──────────────────────────────────────────────────────────
    memset(&gState, 0, sizeof(gState));
    gState.autoMode              = (gConfig.boot_auto_mode != 0);
    gState.lastGatewayContact_ms = millis();
    gState.lastPondContact_ms    = millis();

    // ── FreeRTOS primitives ───────────────────────────────────────────────────
    Serial.println("[INIT] Creating FreeRTOS primitives...");
    xStateMutex = xSemaphoreCreateMutex();
    configASSERT(xStateMutex != nullptr);

    xI2cMutex = xSemaphoreCreateMutex();
    configASSERT(xI2cMutex != nullptr);

    xLoRaIrqSemaphore = xSemaphoreCreateBinary();
    configASSERT(xLoRaIrqSemaphore != nullptr);

    xTxQueue = xQueueCreate(8, sizeof(LoRaPacket));
    configASSERT(xTxQueue != nullptr);

    xRxQueue = xQueueCreate(8, sizeof(LoRaPacket));
    configASSERT(xRxQueue != nullptr);

    // ── RadioLib SX1262 ───────────────────────────────────────────────────────
    Serial.println("------------------------------------------------------------");
    Serial.printf("[INIT] SX1262 SPI : SCK=GPIO%d  MISO=GPIO%d  MOSI=GPIO%d  NSS=GPIO%d\n",
                  LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    Serial.printf("[INIT] SX1262 ctrl: RST=GPIO%d  BUSY=GPIO%d  DIO1=GPIO%d\n",
                  LORA_NRST, LORA_BUSY, LORA_DIO1);
    Serial.printf("[INIT] RF settings: %.1f MHz  BW=%.0f kHz  SF%d  CR4/%d  PWR=%d dBm\n",
                  LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SF, LORA_CR, LORA_TX_POWER);
    Serial.flush();  // ensure RF settings reach the monitor before any potentially-hanging operation

    // Crash-streak guard: if the last RADIO_SKIP_STREAK boots ended in INT_WDT or
    // Panic (likely during radio init), skip radio init entirely this boot and let
    // the user recover via standalone mode.  Cleared on a successful radio boot.
    bool skipRadioInit = false;
    if (gRadioCrashStreak >= RADIO_SKIP_STREAK) {
        skipRadioInit = true;
        Serial.printf("[WARN] Radio init skipped — %u consecutive crash-boots detected.\n", gRadioCrashStreak);
        Serial.println("[WARN] Booting in forced standalone mode.  Reset the crash counter by");
        Serial.println("[WARN] erasing NVS (pio run -t erase) or repairing the SX1262 wiring.");
    }

    // Increment streak now; reset to 0 after init completes (success or soft-failure).
    // A crash (INT_WDT/Panic) before the reset leaves the counter elevated in NVS.
    if (!skipRadioInit) {
        Preferences p;
        p.begin(NVS_NAMESPACE, false);
        p.putUChar(NVS_RADIO_STREAK_KEY, (uint8_t)(gRadioCrashStreak + 1));
        p.end();
    }

    if (!skipRadioInit) {
        // Pre-check BUSY with a pull-up so a floating (absent/unpowered) module reads HIGH.
        // The SX1262 actively drives BUSY LOW when idle, overriding the pull-up.
        // Without the pull-up a floating pin can read LOW and enter a crashing init path.
        pinMode(LORA_BUSY, INPUT_PULLUP);
        delayMicroseconds(200);  // let pull-up settle
        bool busyLow = (digitalRead(LORA_BUSY) == LOW);
        pinMode(LORA_BUSY, INPUT);  // release pull-up; RadioLib reconfigures during begin()

        if (!busyLow) {
            Serial.println("[WARN] LORA_BUSY not driven LOW → SX1262 absent, unpowered, or wiring error.");
            Serial.println("[WARN] Skipping radio init.  Check: module power, RST/BUSY/SPI wiring.");
            Serial.flush();
        } else {
            Serial.println("[INIT] LORA_BUSY is LOW → SX1262 detected. Initialising SPI...");
            Serial.flush();

            loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

            Serial.println("[INIT] SPI ready.  Calling radio.begin()...");
            Serial.flush();

            int radioState = radio.begin(
                LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SF,
                LORA_CR, LORA_SYNC_WORD, LORA_TX_POWER, LORA_PREAMBLE);

            if (radioState != RADIOLIB_ERR_NONE) {
                Serial.printf("[ERROR] radio.begin() failed — RadioLib code %d\n", radioState);
                Serial.println("[ERROR] Possible causes:");
                Serial.println("[ERROR]   • SPI wiring error  (SCK / MISO / MOSI / NSS)");
                Serial.println("[ERROR]   • RST or BUSY pin disconnected or shorted");
                Serial.println("[ERROR]   • Module not powered or damaged");
            } else {
                radio.setCRC(true);
                radio.setPacketReceivedAction(onPacketReceived);
                radioState = radio.startReceive();
                if (radioState != RADIOLIB_ERR_NONE) {
                    Serial.printf("[ERROR] radio.startReceive() failed — RadioLib code %d\n", radioState);
                } else {
                    gRadioOk = true;
                    Serial.printf("[BOOT] Radio OK  |  %.1f MHz  |  Packet size: %u bytes\n",
                                  LORA_FREQUENCY, (unsigned)sizeof(LoRaPacket));
                }
            }
        }
        // We reached this point without crashing — reset streak regardless of success/failure.
        // The streak only stays elevated if the device crashes (INT_WDT/Panic) before reaching here.
        {
            Preferences p;
            p.begin(NVS_NAMESPACE, false);
            p.putUChar(NVS_RADIO_STREAK_KEY, 0);
            p.end();
        }
    }

    if (!gRadioOk) {
        Serial.println("[WARN] *** STANDALONE MODE — LoRa disabled ***");
        Serial.println("[WARN]     Local pumps P1/P2/P3, float switches, buttons, LEDs: fully operational.");
        Serial.println("[WARN]     Pond pump and gateway communication: disabled.");
        Serial.println("[WARN]     Fix hardware and reset to enable LoRa.");
        Serial.flush();
        // Note: no FastLED.show() here — FastLED's legacy RMT backend on ESP32-S3
        // disables CPU interrupts during the busy-wait and can exceed the 300 ms
        // INT_WDT threshold.  UIAnimation (Core 1, 30 ms tick) will light the LEDs
        // as soon as the task scheduler starts.
    }
    Serial.println("------------------------------------------------------------");

    // ── FreeRTOS tasks ────────────────────────────────────────────────────────
    Serial.println("[INIT] Starting FreeRTOS tasks..."); Serial.flush();
    BaseType_t rc;

    // Core 1: InputSensorPoll P3, ControlEngine P4, UIAnimation P1
    //   FastLED RMT driver is initialised in setup() which runs on Core 1.
    //   Calling FastLED.show() from Core 0 can block the RMT wait with interrupts
    //   disabled long enough to trigger INT_WDT — keep UIAnimation on Core 1.
    // Core 0: LoRaTransceiver P2 (event-driven, no FastLED)
    rc = xTaskCreatePinnedToCore(Task_InputSensorPoll, "InputPoll", 4096, nullptr, 3, nullptr, 1);
    configASSERT(rc == pdPASS);
    Serial.println("[INIT]   InputSensorPoll  → Core 1  Priority 3  OK"); Serial.flush();

    rc = xTaskCreatePinnedToCore(Task_ControlEngine,   "CtrlEng",  8192, nullptr, 4, nullptr, 1);
    configASSERT(rc == pdPASS);
    Serial.println("[INIT]   ControlEngine    → Core 1  Priority 4  OK"); Serial.flush();

    rc = xTaskCreatePinnedToCore(Task_LoRaTransceiver, "LoRaTx",   8192, nullptr, 2, nullptr, 0);
    configASSERT(rc == pdPASS);
    Serial.println("[INIT]   LoRaTransceiver  → Core 0  Priority 2  OK"); Serial.flush();

    rc = xTaskCreatePinnedToCore(Task_UIAnimation,     "UIAnim",   8192, nullptr, 1, nullptr, 1);
    configASSERT(rc == pdPASS);
    Serial.println("[INIT]   UIAnimation      → Core 1  Priority 1  OK"); Serial.flush();

    Serial.println("============================================================");
    Serial.println("[BOOT] System operational.");
    Serial.println("============================================================");
}

// =============================================================================
// SECTION 17 – LOOP (all work is in RTOS tasks)
// =============================================================================

void loop() { vTaskDelay(portMAX_DELAY); }

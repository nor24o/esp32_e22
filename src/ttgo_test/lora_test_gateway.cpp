// =============================================================================
// LORA DUAL-ROLE COMMUNICATION TEST — with Web Dashboard
// Target  : TTGO LoRa32 V1.0 (ESP32 + SX1276 HPD13A)
// Purpose : Test both communication paths of the tank controller system.
//
// ROLE A — GATEWAY  (NODE_GATEWAY  0x01)
//   Simulates the central gateway / home automation controller.
//   • Sends pump commands with force-override to the tank node (0x02)
//   • Sends CONFIG_GET / STATS_GET and decodes the responses
//   • Periodically pings the tank so its network-timeout counter stays clear
//
// ROLE B — POND NODE  (NODE_POND_REMOTE 0x03)
//   Simulates the remote pond pump controller.
//   • Receives pump ON/OFF commands from the tank node (0x02)
//   • Sends ACK for every received command (required by tank's retry logic)
//   • Sends periodic telemetry so the tank's lastPondContact_ms stays fresh
//   • Prints commanded pond-pump state so you can verify the tank logic
//
// Switch role at runtime via serial OR web interface — no reflash needed.
//
// TTGO V1 SX1276 pin map:
//   SCK=5  MISO=19  MOSI=27  NSS=18  RST=14  DIO0=26
//
// RF settings (must match tank node exactly):
//   868.0 MHz | BW 125 kHz | SF9 | CR 4/5 | Sync 0x12 | CRC on
//
// Web Interface:
//   WiFi AP: "LoRaTest-TTGO" / "loratest1"
//   URL    : http://192.168.4.1
//   WS     : ws://192.168.4.1/ws
//
// Serial commands (115200 baud) — active in both roles unless noted:
//   g   — switch to GATEWAY role
//   p   — switch to POND NODE role
//   h   — print help / current role
//
//   [GATEWAY only]
//   c   — MSG_CONFIG_GET
//   s   — MSG_STATS_GET
//   1/! — Pump 1 ON / OFF  (force-override)
//   2/@ — Pump 2 ON / OFF
//   3/# — Pump 3 ON / OFF
//   k   — send keepalive ping now (CONFIG_GET)
//
//   [POND NODE only]
//   t   — send telemetry to tank now (also sent automatically every 25 s)
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "web_page.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// =============================================================================
// TTGO V1 PIN MAP
// =============================================================================

#define LORA_NSS   18
#define LORA_DIO0  26   // SX1276 uses DIO0 for RxDone/TxDone
#define LORA_NRST  14
#define LORA_DIO1  -1
#define LORA_SCK    5
#define LORA_MISO  19
#define LORA_MOSI  27

// =============================================================================
// RF SETTINGS
// =============================================================================

#define LORA_FREQUENCY   868.0f
#define LORA_BANDWIDTH   125.0f
#define LORA_SF          9
#define LORA_CR          5
#define LORA_SYNC_WORD   0x12
#define LORA_TX_POWER    17
#define LORA_PREAMBLE    8

// Keepalive / telemetry intervals
#define GATEWAY_KEEPALIVE_MS   30000UL  // send CONFIG_GET to tank every 30 s
#define POND_TELEMETRY_MS      25000UL  // send telemetry to tank every 25 s
#define STATUS_BCAST_MS         5000UL  // push status JSON every 5 s

// =============================================================================
// PROTOCOL (keep in sync with include/protocol.hpp on the tank)
// =============================================================================

#define MY_NETWORK_MAGIC  0x5A6B
#define NODE_GATEWAY      0x01
#define NODE_TANK_LOCAL   0x02
#define NODE_POND_REMOTE  0x03
#define NODE_BROADCAST    0xFF

enum MessageType : uint8_t {
    MSG_TELEMETRY   = 1,
    MSG_COMMAND     = 2,
    MSG_ACK         = 3,
    MSG_CONFIG_GET  = 4,
    MSG_CONFIG_SET  = 5,
    MSG_CONFIG_RESP = 6,
    MSG_STATS_GET   = 7,
    MSG_STATS_RESP  = 8,
};

struct __attribute__((packed)) LoRaHeader {
    uint16_t magic_word;
    uint8_t  target_id;
    uint8_t  sender_id;
    uint8_t  msg_id;
    uint8_t  msg_type;
};

struct __attribute__((packed)) TelemetryData {
    uint8_t  water_level;
    uint8_t  system_flags;  // bit0=Auto bit1=P1 bit2=P2 bit3=P3 bit4=PondP
    int16_t  temperature;   // °C × 10
    uint8_t  humidity;
    uint8_t  error_code;    // 0=OK 1=OC 2=DryRun 3=Comms
    int8_t   last_rssi;
    int8_t   last_snr;
};

struct __attribute__((packed)) CommandData {
    uint8_t target_pump;    // 1-4; also acked-msg-id in ACK frames
    uint8_t action;         // 0=Off 1=On
    uint8_t flags;          // bit0=Force Override
};

struct __attribute__((packed)) NodeConfig {
    uint32_t pump_min_runtime_ms;
    uint32_t pump_min_cooldown_ms;
    uint32_t replenish_runon_ms;
    uint32_t telemetry_interval_ms;
    uint32_t network_timeout_ms;
    uint32_t ack_timeout_ms;
    uint16_t overcurrent_thresh;
    uint16_t dryrun_thresh;
    uint8_t  ack_max_retries;
    uint8_t  boot_auto_mode;
    uint8_t  overcurrent_grace_ticks;
    uint8_t  fault_lockout_enabled;
};

struct __attribute__((packed)) StatsPayload {
    uint32_t uptime_s;
    uint32_t runtime_p1_s;
    uint32_t runtime_p2_s;
    uint32_t runtime_p3_s;
    uint32_t runtime_pond_s;
    uint16_t fill_cycles;
    uint16_t boot_count;
    uint8_t  last_fault;
    uint8_t  reserved[7];
};

struct __attribute__((packed)) LoRaPacket {
    LoRaHeader header;
    union {
        TelemetryData telemetry;
        CommandData   command;
        NodeConfig    config;
        StatsPayload  stats;
    } payload;
};

static_assert(sizeof(LoRaPacket) <= 255, "LoRaPacket too large");

// =============================================================================
// RADIO
// =============================================================================

static SPIClass        loraSPI(VSPI);
static SX1276          radio = new Module(LORA_NSS, LORA_DIO0, LORA_NRST, LORA_DIO1, loraSPI);
static volatile bool   rxFlag = false;

void IRAM_ATTR onRxDone() { rxFlag = true; }

// =============================================================================
// WEB SERVER / WEBSOCKET
// =============================================================================

static AsyncWebServer  server(80);
static AsyncWebSocket  ws("/ws");

// JSON scratch buffer — not re-entrant, only used from loop() (main core)
static char gJsonBuf[600];

// WebSocket command inbox — written from WS callback, read in loop()
static char              gWsCmd[256];
static volatile bool     gWsCmdReady = false;

static void broadcastJson(const char *json) {
    ws.textAll(json);
}

// =============================================================================
// RUNTIME STATE
// =============================================================================

enum NodeRole : uint8_t { ROLE_GATEWAY = 0, ROLE_POND = 1 };
static NodeRole  gRole      = ROLE_GATEWAY;
static uint8_t   gMsgId     = 0;
static int8_t    gLastRssi  = 0;
static int8_t    gLastSnr   = 0;

// Pond-node state (used in ROLE_POND only)
static bool      gPondPumpOn = false;  // commanded state from tank

// Timers
static uint32_t  gLastKeepalive_ms  = 0;
static uint32_t  gLastTelemetry_ms  = 0;
static uint32_t  gLastStatusBcast   = 0;

// =============================================================================
// JSON BROADCAST HELPERS
// =============================================================================

static void bcastLog(const char *lvl, const char *msg) {
    // lvl: "rx" "tx" "err" "inf" "cmd"
    snprintf(gJsonBuf, sizeof(gJsonBuf),
             "{\"t\":\"log\",\"lvl\":\"%s\",\"msg\":\"%s\"}", lvl, msg);
    broadcastJson(gJsonBuf);
}

static void bcastRole() {
    const char *r = (gRole == ROLE_GATEWAY) ? "gateway" : "pond";
    snprintf(gJsonBuf, sizeof(gJsonBuf), "{\"t\":\"role\",\"role\":\"%s\"}", r);
    broadcastJson(gJsonBuf);
}

static void bcastStatus() {
    const char *r = (gRole == ROLE_GATEWAY) ? "gateway" : "pond";
    snprintf(gJsonBuf, sizeof(gJsonBuf),
             "{\"t\":\"status\",\"role\":\"%s\",\"heap\":%lu,\"uptime\":%lu,\"clients\":%u}",
             r, (unsigned long)ESP.getFreeHeap(),
             (unsigned long)(millis() / 1000),
             (unsigned)ws.count());
    broadcastJson(gJsonBuf);
}

static void bcastTx(uint8_t msgType, uint8_t targetId, uint8_t msgId) {
    snprintf(gJsonBuf, sizeof(gJsonBuf),
             "{\"t\":\"tx\",\"mtype\":%u,\"target\":%u,\"id\":%u}",
             msgType, targetId, msgId);
    broadcastJson(gJsonBuf);
}

static void bcastTelem(const TelemetryData &t, int8_t rssi, int8_t snr) {
    snprintf(gJsonBuf, sizeof(gJsonBuf),
             "{\"t\":\"telem\","
             "\"wl\":%u,\"auto\":%u,\"p1\":%u,\"p2\":%u,\"p3\":%u,\"pond\":%u,"
             "\"tmp\":%d,\"hum\":%u,\"err\":%u,"
             "\"nrssi\":%d,\"nsnr\":%d,\"rssi\":%d,\"snr\":%d}",
             t.water_level,
             (t.system_flags & 0x01) ? 1 : 0,
             (t.system_flags & 0x02) ? 1 : 0,
             (t.system_flags & 0x04) ? 1 : 0,
             (t.system_flags & 0x08) ? 1 : 0,
             (t.system_flags & 0x10) ? 1 : 0,
             (int)t.temperature, (unsigned)t.humidity, (unsigned)t.error_code,
             (int)t.last_rssi, (int)t.last_snr,
             (int)rssi, (int)snr);
    broadcastJson(gJsonBuf);
}

static void bcastCfg(const NodeConfig &c, int8_t rssi, int8_t snr) {
    snprintf(gJsonBuf, sizeof(gJsonBuf),
             "{\"t\":\"cfg\","
             "\"pmr\":%lu,\"pmc\":%lu,\"pro\":%lu,\"ti\":%lu,\"nt\":%lu,\"at\":%lu,"
             "\"oct\":%u,\"drt\":%u,\"amr\":%u,\"bam\":%u,\"ocg\":%u,\"fle\":%u,"
             "\"rssi\":%d,\"snr\":%d}",
             (unsigned long)c.pump_min_runtime_ms,
             (unsigned long)c.pump_min_cooldown_ms,
             (unsigned long)c.replenish_runon_ms,
             (unsigned long)c.telemetry_interval_ms,
             (unsigned long)c.network_timeout_ms,
             (unsigned long)c.ack_timeout_ms,
             (unsigned)c.overcurrent_thresh,
             (unsigned)c.dryrun_thresh,
             (unsigned)c.ack_max_retries,
             (unsigned)c.boot_auto_mode,
             (unsigned)c.overcurrent_grace_ticks,
             (unsigned)c.fault_lockout_enabled,
             (int)rssi, (int)snr);
    broadcastJson(gJsonBuf);
}

static void bcastStats(const StatsPayload &s, int8_t rssi, int8_t snr) {
    snprintf(gJsonBuf, sizeof(gJsonBuf),
             "{\"t\":\"stats\","
             "\"up\":%lu,\"rt1\":%lu,\"rt2\":%lu,\"rt3\":%lu,\"rtp\":%lu,"
             "\"fc\":%u,\"bc\":%u,\"lf\":%u,"
             "\"rssi\":%d,\"snr\":%d}",
             (unsigned long)s.uptime_s,
             (unsigned long)s.runtime_p1_s,
             (unsigned long)s.runtime_p2_s,
             (unsigned long)s.runtime_p3_s,
             (unsigned long)s.runtime_pond_s,
             (unsigned)s.fill_cycles,
             (unsigned)s.boot_count,
             (unsigned)s.last_fault,
             (int)rssi, (int)snr);
    broadcastJson(gJsonBuf);
}

static void bcastAck(uint8_t ackedId, int8_t rssi, int8_t snr) {
    snprintf(gJsonBuf, sizeof(gJsonBuf),
             "{\"t\":\"ack\",\"id\":%u,\"rssi\":%d,\"snr\":%d}",
             (unsigned)ackedId, (int)rssi, (int)snr);
    broadcastJson(gJsonBuf);
}

static void bcastCmd(uint8_t pump, uint8_t action, uint8_t msgId, int8_t rssi, int8_t snr) {
    snprintf(gJsonBuf, sizeof(gJsonBuf),
             "{\"t\":\"cmd\",\"pump\":%u,\"action\":%u,\"id\":%u,\"rssi\":%d,\"snr\":%d}",
             (unsigned)pump, (unsigned)action, (unsigned)msgId, (int)rssi, (int)snr);
    broadcastJson(gJsonBuf);
}

// =============================================================================
// WEBSOCKET EVENT HANDLER
// =============================================================================

static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[WS] Client #%u connected  IP=%s\n",
                      client->id(), client->remoteIP().toString().c_str());
        // Send current role and status immediately
        bcastRole();
        bcastStatus();
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WS] Client #%u disconnected\n", client->id());
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len &&
            info->opcode == WS_TEXT && !gWsCmdReady)
        {
            size_t copy = (len < sizeof(gWsCmd) - 1) ? len : sizeof(gWsCmd) - 1;
            memcpy(gWsCmd, data, copy);
            gWsCmd[copy] = '\0';
            gWsCmdReady  = true;
        }
    }
}

// =============================================================================
// TRANSMIT HELPER
// =============================================================================

static void sendPacket(LoRaPacket &pkt, uint8_t target, uint8_t msgType) {
    uint8_t senderId = (gRole == ROLE_GATEWAY) ? NODE_GATEWAY : NODE_POND_REMOTE;
    pkt.header.magic_word = MY_NETWORK_MAGIC;
    pkt.header.target_id  = target;
    pkt.header.sender_id  = senderId;
    pkt.header.msg_id     = ++gMsgId;
    pkt.header.msg_type   = msgType;

    int state = radio.transmit((uint8_t *)&pkt, sizeof(LoRaPacket));
    if (state == RADIOLIB_ERR_NONE) {
        Serial.printf("[TX] type=%u  id=%u  to=0x%02X  from=0x%02X  OK\n",
                      msgType, gMsgId, target, senderId);
        bcastTx(msgType, target, gMsgId);
    } else {
        Serial.printf("[TX] FAILED state=%d\n", state);
        char tmp[64];
        snprintf(tmp, sizeof(tmp), "TX FAILED state=%d", state);
        bcastLog("err", tmp);
    }
    radio.startReceive();
}

// =============================================================================
// GATEWAY ACTIONS
// =============================================================================

static void gw_sendConfigGet() {
    LoRaPacket pkt; memset(&pkt, 0, sizeof(pkt));
    sendPacket(pkt, NODE_TANK_LOCAL, MSG_CONFIG_GET);
}

static void gw_sendStatsGet() {
    LoRaPacket pkt; memset(&pkt, 0, sizeof(pkt));
    sendPacket(pkt, NODE_TANK_LOCAL, MSG_STATS_GET);
}

static void gw_sendPumpCommand(uint8_t pump, uint8_t action) {
    LoRaPacket pkt; memset(&pkt, 0, sizeof(pkt));
    pkt.payload.command.target_pump = pump;
    pkt.payload.command.action      = action;
    pkt.payload.command.flags       = 0x01;  // force-override auto mode
    sendPacket(pkt, NODE_TANK_LOCAL, MSG_COMMAND);
}

// =============================================================================
// POND NODE ACTIONS
// =============================================================================

static void pond_sendAck(uint8_t ackedMsgId) {
    LoRaPacket pkt; memset(&pkt, 0, sizeof(pkt));
    pkt.payload.command.target_pump = ackedMsgId;  // convention: acked id goes here
    pkt.payload.command.action      = 0;
    pkt.payload.command.flags       = 0;
    sendPacket(pkt, NODE_TANK_LOCAL, MSG_ACK);
}

static void pond_sendTelemetry() {
    LoRaPacket pkt; memset(&pkt, 0, sizeof(pkt));
    // Simulated pond telemetry — system_flags bit4 reflects commanded pump state
    pkt.payload.telemetry.water_level  = 2;    // pond always has water
    pkt.payload.telemetry.system_flags = gPondPumpOn ? (1 << 4) : 0;
    pkt.payload.telemetry.temperature  = 210;  // 21.0 °C placeholder
    pkt.payload.telemetry.humidity     = 65;
    pkt.payload.telemetry.error_code   = 0;
    pkt.payload.telemetry.last_rssi    = gLastRssi;
    pkt.payload.telemetry.last_snr     = gLastSnr;
    sendPacket(pkt, NODE_TANK_LOCAL, MSG_TELEMETRY);
    gLastTelemetry_ms = millis();
    Serial.printf("[POND] Telemetry sent  pumpOn=%s\n", gPondPumpOn ? "YES" : "no");
}

// =============================================================================
// PACKET DECODER + WS BROADCAST
// =============================================================================

static void decodeAndPrint(const LoRaPacket *pkt, int8_t rssi, int8_t snr) {
    gLastRssi = rssi;
    gLastSnr  = snr;

    Serial.println("┌──────────────────────────────────────────────────");
    Serial.printf( "│ RX  from=0x%02X  type=%u  id=%u  RSSI=%d dBm  SNR=%d dB\n",
                   pkt->header.sender_id, pkt->header.msg_type,
                   pkt->header.msg_id, rssi, snr);

    switch (pkt->header.msg_type) {

        case MSG_TELEMETRY: {
            const TelemetryData &t = pkt->payload.telemetry;
            const char *errStr =
                t.error_code == 0 ? "OK"         :
                t.error_code == 1 ? "OVERCURRENT":
                t.error_code == 2 ? "DRY-RUN"    :
                t.error_code == 3 ? "NO-COMMS"   : "UNKNOWN";
            Serial.println("│ MSG_TELEMETRY");
            Serial.printf( "│   Water level : %u/3\n",     t.water_level);
            Serial.printf( "│   Mode        : %s\n",       (t.system_flags & 0x01) ? "AUTO" : "MANUAL");
            Serial.printf( "│   Pumps       : P1=%s P2=%s P3=%s Pond=%s\n",
                           (t.system_flags & 0x02) ? "ON" : "off",
                           (t.system_flags & 0x04) ? "ON" : "off",
                           (t.system_flags & 0x08) ? "ON" : "off",
                           (t.system_flags & 0x10) ? "ON" : "off");
            Serial.printf( "│   Temp        : %.1f °C   Humidity: %u%%\n",
                           t.temperature / 10.0f, t.humidity);
            Serial.printf( "│   Error       : %s (%u)\n",  errStr, t.error_code);
            Serial.printf( "│   Node RSSI   : %d dBm  SNR: %d dB\n", t.last_rssi, t.last_snr);
            bcastTelem(t, rssi, snr);
            break;
        }

        case MSG_COMMAND: {
            const CommandData &cmd = pkt->payload.command;
            Serial.println("│ MSG_COMMAND");
            Serial.printf( "│   Pump        : %u\n",  cmd.target_pump);
            Serial.printf( "│   Action      : %s\n",  cmd.action ? "ON" : "OFF");
            Serial.printf( "│   Force-ovr   : %s\n",  (cmd.flags & 0x01) ? "yes" : "no");
            bcastCmd(cmd.target_pump, cmd.action, pkt->header.msg_id, rssi, snr);

            // In pond role: apply command and ACK immediately
            if (gRole == ROLE_POND && cmd.target_pump == 1) {
                gPondPumpOn = (cmd.action == 1);
                Serial.printf("│   → Pond pump commanded %s — sending ACK\n",
                              gPondPumpOn ? "ON" : "OFF");
                pond_sendAck(pkt->header.msg_id);
            }
            break;
        }

        case MSG_ACK: {
            uint8_t ackedId = pkt->payload.command.target_pump;
            Serial.println("│ MSG_ACK");
            Serial.printf( "│   Acked id    : %u\n", ackedId);
            bcastAck(ackedId, rssi, snr);
            break;
        }

        case MSG_CONFIG_RESP: {
            const NodeConfig &c = pkt->payload.config;
            Serial.println("│ MSG_CONFIG_RESP");
            Serial.printf( "│   pump_min_runtime    : %lu ms\n",  c.pump_min_runtime_ms);
            Serial.printf( "│   pump_min_cooldown   : %lu ms\n",  c.pump_min_cooldown_ms);
            Serial.printf( "│   replenish_runon     : %lu ms\n",  c.replenish_runon_ms);
            Serial.printf( "│   telemetry_interval  : %lu ms\n",  c.telemetry_interval_ms);
            Serial.printf( "│   network_timeout     : %lu ms\n",  c.network_timeout_ms);
            Serial.printf( "│   ack_timeout         : %lu ms\n",  c.ack_timeout_ms);
            Serial.printf( "│   overcurrent_thresh  : %u  dryrun_thresh: %u\n",
                           c.overcurrent_thresh, c.dryrun_thresh);
            Serial.printf( "│   ack_max_retries     : %u\n",      c.ack_max_retries);
            Serial.printf( "│   boot_auto_mode      : %u\n",      c.boot_auto_mode);
            Serial.printf( "│   oc_grace_ticks      : %u (x50ms)\n", c.overcurrent_grace_ticks);
            Serial.printf( "│   fault_lockout       : %s\n",
                           c.fault_lockout_enabled ? "ENABLED" : "warn-only");
            bcastCfg(c, rssi, snr);
            break;
        }

        case MSG_STATS_RESP: {
            const StatsPayload &s = pkt->payload.stats;
            Serial.println("│ MSG_STATS_RESP");
            Serial.printf( "│   Uptime       : %lu s (%lu min)\n", s.uptime_s, s.uptime_s / 60);
            Serial.printf( "│   P1 runtime   : %lu s\n",  s.runtime_p1_s);
            Serial.printf( "│   P2 runtime   : %lu s\n",  s.runtime_p2_s);
            Serial.printf( "│   P3 runtime   : %lu s\n",  s.runtime_p3_s);
            Serial.printf( "│   Pond runtime : %lu s\n",  s.runtime_pond_s);
            Serial.printf( "│   Fill cycles  : %u\n",     s.fill_cycles);
            Serial.printf( "│   Boot count   : %u\n",     s.boot_count);
            Serial.printf( "│   Last fault   : %u\n",     s.last_fault);
            bcastStats(s, rssi, snr);
            break;
        }

        default:
            Serial.printf("│ Unknown msg_type=%u\n", pkt->header.msg_type);
            break;
    }
    Serial.println("└──────────────────────────────────────────────────");
}

// =============================================================================
// WEBSOCKET COMMAND PROCESSOR
// =============================================================================

static void processWsCommand(const char *json) {
    // Minimal key extraction with strstr — no JSON lib dependency
    if (strstr(json, "\"cmd\":\"role\"")) {
        if (strstr(json, "\"pond\"")) {
            Serial.println("[WS] Role → POND");
            gRole             = ROLE_POND;
            gLastTelemetry_ms = millis();
            gPondPumpOn       = false;
            bcastRole();
            bcastLog("inf", "Role switched to POND");
        } else if (strstr(json, "\"gateway\"")) {
            Serial.println("[WS] Role → GATEWAY");
            gRole               = ROLE_GATEWAY;
            gLastKeepalive_ms   = millis();
            bcastRole();
            bcastLog("inf", "Role switched to GATEWAY");
        }
    } else if (strstr(json, "\"cmd\":\"pump\"")) {
        if (gRole != ROLE_GATEWAY) { bcastLog("err", "Must be GATEWAY to send pump commands"); return; }
        // Parse "p" and "a" fields: {"cmd":"pump","p":1,"a":1}
        const char *pp = strstr(json, "\"p\":");
        const char *ap = strstr(json, "\"a\":");
        if (pp && ap) {
            uint8_t pump   = (uint8_t)atoi(pp + 4);
            uint8_t action = (uint8_t)atoi(ap + 4);
            if (pump >= 1 && pump <= 3) {
                gw_sendPumpCommand(pump, action);
            }
        }
    } else if (strstr(json, "\"cmd\":\"cfg_get\"")) {
        if (gRole != ROLE_GATEWAY) { bcastLog("err", "Must be GATEWAY for cfg_get"); return; }
        gw_sendConfigGet();
    } else if (strstr(json, "\"cmd\":\"stats_get\"")) {
        if (gRole != ROLE_GATEWAY) { bcastLog("err", "Must be GATEWAY for stats_get"); return; }
        gw_sendStatsGet();
    } else if (strstr(json, "\"cmd\":\"keepalive\"")) {
        if (gRole != ROLE_GATEWAY) { bcastLog("err", "Must be GATEWAY for keepalive"); return; }
        gw_sendConfigGet();
        gLastKeepalive_ms = millis();
    } else if (strstr(json, "\"cmd\":\"telemetry\"")) {
        if (gRole != ROLE_POND) { bcastLog("err", "Must be POND for telemetry"); return; }
        pond_sendTelemetry();
    }
}

// =============================================================================
// HELP / ROLE PRINT
// =============================================================================

static void printHelp() {
    Serial.println();
    const char *roleName = (gRole == ROLE_GATEWAY) ? "GATEWAY (0x01)" : "POND NODE (0x03)";
    Serial.printf("=== LoRa Dual-Role Test | Current role: %s ===\n", roleName);
    Serial.printf("=== Web UI: http://192.168.4.1  (SSID: LoRaTest-TTGO) ===\n");
    Serial.println("  g   — switch to GATEWAY role  (NODE_GATEWAY  0x01)");
    Serial.println("  p   — switch to POND NODE role (NODE_POND_REMOTE 0x03)");
    Serial.println("  h   — this help");
    if (gRole == ROLE_GATEWAY) {
        Serial.println("  --- Gateway commands ---");
        Serial.println("  c   — MSG_CONFIG_GET");
        Serial.println("  s   — MSG_STATS_GET");
        Serial.println("  1/! — Pump 1 ON/OFF (force-override)");
        Serial.println("  2/@ — Pump 2 ON/OFF");
        Serial.println("  3/# — Pump 3 ON/OFF");
        Serial.println("  k   — send keepalive ping now");
    } else {
        Serial.println("  --- Pond node commands ---");
        Serial.println("  t   — send telemetry to tank now");
        Serial.println("  (pump commands from tank are ACKed automatically)");
    }
    Serial.println("=================================================");
    Serial.println();
}

static void switchRole(NodeRole newRole) {
    gRole               = newRole;
    gLastKeepalive_ms   = millis();
    gLastTelemetry_ms   = millis();
    gPondPumpOn         = false;
    const char *name    = (newRole == ROLE_GATEWAY) ? "GATEWAY (0x01)" : "POND NODE (0x03)";
    Serial.printf("\n[ROLE] Switched to %s\n", name);
    bcastRole();
    bcastLog("inf", newRole == ROLE_GATEWAY ? "Role switched to GATEWAY" : "Role switched to POND");
    printHelp();
}

// =============================================================================
// WIFI + WEB SERVER INIT
// =============================================================================

static void setupWiFi() {
    WiFi.softAP("LoRaTest-TTGO", "loratest1");
    IPAddress ip = WiFi.softAPIP();
    Serial.printf("[WIFI] AP started  SSID=LoRaTest-TTGO  IP=%s\n", ip.toString().c_str());
}

static void setupWebServer() {
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send_P(200, "text/html", INDEX_HTML);
    });

    server.begin();
    Serial.println("[HTTP] Web server started on port 80");
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n============================================================");
    Serial.println("[BOOT] LoRa Dual-Role Test — TTGO LoRa32 V1.0 (SX1276)");
    Serial.printf( "[INIT] %.1f MHz  BW=%.0f kHz  SF%d  CR4/%d  Sync=0x%02X  PWR=%d dBm\n",
                   LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SF, LORA_CR,
                   LORA_SYNC_WORD, LORA_TX_POWER);
    Serial.printf( "[INIT] SPI: SCK=%d MISO=%d MOSI=%d NSS=%d RST=%d DIO0=%d\n",
                   LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS, LORA_NRST, LORA_DIO0);
    Serial.println("------------------------------------------------------------");

    setupWiFi();
    setupWebServer();

    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    int state = radio.begin(
        LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SF,
        LORA_CR, LORA_SYNC_WORD, LORA_TX_POWER, LORA_PREAMBLE);

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] radio.begin() failed — code %d\n", state);
        Serial.println("[ERROR] Check: module power, SPI wiring (SCK/MISO/MOSI/NSS), RST, DIO0.");
        Serial.println("[ERROR] Halting.");
        while (true) delay(1000);
    }

    radio.setCRC(true);
    radio.setPacketReceivedAction(onRxDone);

    state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] startReceive() failed — code %d\n", state);
        while (true) delay(1000);
    }

    Serial.printf("[BOOT] SX1276 OK  |  packet size: %u bytes\n", (unsigned)sizeof(LoRaPacket));
    Serial.println("============================================================");

    gLastKeepalive_ms = millis();
    gLastTelemetry_ms = millis();
    gLastStatusBcast  = millis();
    printHelp();
}

// =============================================================================
// LOOP
// =============================================================================

void loop() {
    uint32_t now = millis();

    // ── WebSocket command from browser ────────────────────────────────────────
    if (gWsCmdReady) {
        gWsCmdReady = false;
        Serial.printf("[WS] cmd: %s\n", gWsCmd);
        processWsCommand(gWsCmd);
    }

    // ── WebSocket housekeeping ────────────────────────────────────────────────
    ws.cleanupClients();

    // ── Periodic status broadcast ─────────────────────────────────────────────
    if ((now - gLastStatusBcast) >= STATUS_BCAST_MS) {
        gLastStatusBcast = now;
        bcastStatus();
    }

    // ── Received LoRa packet ──────────────────────────────────────────────────
    if (rxFlag) {
        rxFlag = false;

        int rxLen = radio.getPacketLength();
        if (rxLen == (int)sizeof(LoRaPacket)) {
            uint8_t buf[sizeof(LoRaPacket)];
            int state = radio.readData(buf, sizeof(LoRaPacket));
            if (state == RADIOLIB_ERR_NONE) {
                int8_t rssi = (int8_t)radio.getRSSI();
                int8_t snr  = (int8_t)radio.getSNR();
                LoRaPacket *pkt = reinterpret_cast<LoRaPacket *>(buf);

                uint8_t myId = (gRole == ROLE_GATEWAY) ? NODE_GATEWAY : NODE_POND_REMOTE;
                bool forMe = (pkt->header.magic_word == MY_NETWORK_MAGIC) &&
                             (pkt->header.target_id  == myId ||
                              pkt->header.target_id  == NODE_BROADCAST);
                if (forMe) {
                    decodeAndPrint(pkt, rssi, snr);
                } else {
                    Serial.printf("[RX] Ignored — magic=0x%04X target=0x%02X (my id=0x%02X)\n",
                                  pkt->header.magic_word, pkt->header.target_id, myId);
                }
            } else {
                Serial.printf("[RX] readData error: %d\n", state);
                bcastLog("err", "RX readData error");
            }
        } else if (rxLen > 0) {
            uint8_t discard[255];
            radio.readData(discard, (size_t)min(rxLen, 255));
            Serial.printf("[RX] Ignored — length %d (expected %u)\n",
                          rxLen, (unsigned)sizeof(LoRaPacket));
        }
        radio.startReceive();
    }

    // ── Periodic LoRa actions ─────────────────────────────────────────────────
    if (gRole == ROLE_GATEWAY) {
        // Send a CONFIG_GET every 30 s so the tank's lastGatewayContact_ms stays fresh
        if ((now - gLastKeepalive_ms) >= GATEWAY_KEEPALIVE_MS) {
            gLastKeepalive_ms = now;
            Serial.println("[GW] Sending keepalive CONFIG_GET...");
            bcastLog("inf", "Sending keepalive CONFIG_GET...");
            gw_sendConfigGet();
        }
    } else {
        // Send telemetry to tank every 25 s so lastPondContact_ms stays fresh
        if ((now - gLastTelemetry_ms) >= POND_TELEMETRY_MS) {
            pond_sendTelemetry();
        }
    }

    // ── Serial commands ───────────────────────────────────────────────────────
    if (Serial.available()) {
        char cmd = (char)Serial.read();
        switch (cmd) {
            // Role switching — always available
            case 'g': switchRole(ROLE_GATEWAY);  break;
            case 'p': switchRole(ROLE_POND);     break;
            case 'h': case '?': printHelp();     break;

            // Gateway commands
            case 'c':
                if (gRole == ROLE_GATEWAY) { Serial.println("[GW] CONFIG_GET");   gw_sendConfigGet();     }
                else Serial.println("[!] Switch to gateway mode first (press 'g')");
                break;
            case 's':
                if (gRole == ROLE_GATEWAY) { Serial.println("[GW] STATS_GET");    gw_sendStatsGet();      }
                else Serial.println("[!] Switch to gateway mode first (press 'g')");
                break;
            case 'k':
                if (gRole == ROLE_GATEWAY) { Serial.println("[GW] Keepalive");    gw_sendConfigGet(); gLastKeepalive_ms = now; }
                else Serial.println("[!] Switch to gateway mode first (press 'g')");
                break;
            case '1':
                if (gRole == ROLE_GATEWAY) { Serial.println("[GW] Pump 1 ON");    gw_sendPumpCommand(1,1); }
                else Serial.println("[!] Gateway mode only");
                break;
            case '!':
                if (gRole == ROLE_GATEWAY) { Serial.println("[GW] Pump 1 OFF");   gw_sendPumpCommand(1,0); }
                else Serial.println("[!] Gateway mode only");
                break;
            case '2':
                if (gRole == ROLE_GATEWAY) { Serial.println("[GW] Pump 2 ON");    gw_sendPumpCommand(2,1); }
                else Serial.println("[!] Gateway mode only");
                break;
            case '@':
                if (gRole == ROLE_GATEWAY) { Serial.println("[GW] Pump 2 OFF");   gw_sendPumpCommand(2,0); }
                else Serial.println("[!] Gateway mode only");
                break;
            case '3':
                if (gRole == ROLE_GATEWAY) { Serial.println("[GW] Pump 3 ON");    gw_sendPumpCommand(3,1); }
                else Serial.println("[!] Gateway mode only");
                break;
            case '#':
                if (gRole == ROLE_GATEWAY) { Serial.println("[GW] Pump 3 OFF");   gw_sendPumpCommand(3,0); }
                else Serial.println("[!] Gateway mode only");
                break;

            // Pond commands
            case 't':
                if (gRole == ROLE_POND) { pond_sendTelemetry(); }
                else Serial.println("[!] Switch to pond mode first (press 'p')");
                break;

            case '\r': case '\n': break;
            default:
                Serial.printf("[?] Unknown command '%c' — press 'h' for help\n", cmd);
                break;
        }
    }
}

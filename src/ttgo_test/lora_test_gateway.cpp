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
#include "protocol.hpp"   // shared: structs, node IDs, RF settings (freq/BW/SF/CR/sync/preamble)
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
// TTGO-SPECIFIC RF SETTINGS
// =============================================================================

// LORA_TX_POWER is node-specific: TTGO SX1276 max 14 dBm; tank SX1262 uses 22 dBm
#define LORA_TX_POWER    14
// All other RF parameters (frequency, BW, SF, CR, sync word, preamble)
// are inherited from protocol.hpp so they always match the tank.

// Keepalive / telemetry intervals
#define GATEWAY_KEEPALIVE_MS   30000UL  // send CONFIG_GET to tank every 30 s
#define POND_TELEMETRY_MS      25000UL  // send telemetry to tank every 25 s
#define STATUS_BCAST_MS         5000UL  // push status JSON every 5 s

// After any TX the channel is busy: tank ACKs (~330 ms), then relays to pond (~330 ms),
// then TTGO auto-ACKs (~330 ms) = ~1 s total.  Hold off periodic TX for 2.5 s so we
// stay in RX and catch the relay command.
#define RADIO_QUIET_MS         2500UL

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
// Dual-mode: acts as NODE_GATEWAY (0x01) AND NODE_POND_REMOTE (0x03) simultaneously.
// Gateway commands use sender 0x01; pond commands use sender 0x03.
// Receives packets addressed to either ID.
// =============================================================================

static uint8_t   gMsgId     = 0;
static int8_t    gLastRssi  = 0;
static int8_t    gLastSnr   = 0;

// Pond simulation state
static bool      gPondPumpOn = false;  // commanded state from tank

// Timers
static uint32_t  gLastKeepalive_ms  = 0;
static uint32_t  gLastTelemetry_ms  = 0;
static uint32_t  gLastStatusBcast   = 0;
static uint32_t  gQuietUntil_ms     = 0;  // periodic TX suppressed until this timestamp

// =============================================================================
// JSON BROADCAST HELPERS
// =============================================================================

static void bcastLog(const char *lvl, const char *msg) {
    // lvl: "rx" "tx" "err" "inf" "cmd"
    snprintf(gJsonBuf, sizeof(gJsonBuf),
             "{\"t\":\"log\",\"lvl\":\"%s\",\"msg\":\"%s\"}", lvl, msg);
    broadcastJson(gJsonBuf);
}

static void bcastStatus() {
    snprintf(gJsonBuf, sizeof(gJsonBuf),
             "{\"t\":\"status\",\"heap\":%lu,\"uptime\":%lu,\"clients\":%u}",
             (unsigned long)ESP.getFreeHeap(),
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
             "\"amr\":%u,\"bam\":%u,"
             "\"rssi\":%d,\"snr\":%d}",
             (unsigned long)c.pump_min_runtime_ms,
             (unsigned long)c.pump_min_cooldown_ms,
             (unsigned long)c.replenish_runon_ms,
             (unsigned long)c.telemetry_interval_ms,
             (unsigned long)c.network_timeout_ms,
             (unsigned long)c.ack_timeout_ms,
             (unsigned)c.ack_max_retries,
             (unsigned)c.boot_auto_mode,
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

static void sendPacket(LoRaPacket &pkt, uint8_t target, uint8_t msgType, uint8_t senderId) {
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
    // Hold off periodic TX so we stay in RX for the tank's ACK + relay to pond.
    gQuietUntil_ms = millis() + RADIO_QUIET_MS;
}

// =============================================================================
// GATEWAY ACTIONS
// =============================================================================

static void gw_sendConfigGet() {
    LoRaPacket pkt; memset(&pkt, 0, sizeof(pkt));
    sendPacket(pkt, NODE_TANK_LOCAL, MSG_CONFIG_GET, NODE_GATEWAY);
}

static void gw_sendStatsGet() {
    LoRaPacket pkt; memset(&pkt, 0, sizeof(pkt));
    sendPacket(pkt, NODE_TANK_LOCAL, MSG_STATS_GET, NODE_GATEWAY);
}

static void gw_sendConfigSet(const NodeConfig &cfg) {
    LoRaPacket pkt; memset(&pkt, 0, sizeof(pkt));
    pkt.payload.config = cfg;
    sendPacket(pkt, NODE_TANK_LOCAL, MSG_CONFIG_SET, NODE_GATEWAY);
}

static void gw_sendPumpCommand(uint8_t pump, uint8_t action) {
    LoRaPacket pkt; memset(&pkt, 0, sizeof(pkt));
    pkt.payload.command.target_pump = pump;
    pkt.payload.command.action      = action;
    pkt.payload.command.flags       = 0x01;  // force-override auto mode
    sendPacket(pkt, NODE_TANK_LOCAL, MSG_COMMAND, NODE_GATEWAY);
}

// =============================================================================
// POND NODE ACTIONS
// =============================================================================

static void pond_sendAck(uint8_t ackedMsgId) {
    LoRaPacket pkt; memset(&pkt, 0, sizeof(pkt));
    pkt.payload.command.target_pump = ackedMsgId;  // convention: acked id goes here
    pkt.payload.command.action      = 0;
    pkt.payload.command.flags       = 0;
    sendPacket(pkt, NODE_TANK_LOCAL, MSG_ACK, NODE_POND_REMOTE);
}

static void pond_sendTelemetry() {
    LoRaPacket pkt; memset(&pkt, 0, sizeof(pkt));
    pkt.payload.telemetry.water_level  = 2;    // pond always has water
    pkt.payload.telemetry.system_flags = gPondPumpOn ? (1 << 4) : 0;
    pkt.payload.telemetry.temperature  = 210;  // 21.0 °C placeholder
    pkt.payload.telemetry.humidity     = 65;
    pkt.payload.telemetry.error_code   = 0;
    pkt.payload.telemetry.last_rssi    = gLastRssi;
    pkt.payload.telemetry.last_snr     = gLastSnr;
    sendPacket(pkt, NODE_TANK_LOCAL, MSG_TELEMETRY, NODE_POND_REMOTE);
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

            // Auto-ACK when the packet was sent to us as pond node (0x03)
            if (pkt->header.target_id == NODE_POND_REMOTE && cmd.target_pump == 1) {
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
            Serial.printf( "│   ack_max_retries     : %u\n",      c.ack_max_retries);
            Serial.printf( "│   boot_auto_mode      : %u\n",      c.boot_auto_mode);
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
    if (strstr(json, "\"cmd\":\"pump\"")) {
        const char *pp = strstr(json, "\"p\":");
        const char *ap = strstr(json, "\"a\":");
        if (pp && ap) {
            uint8_t pump   = (uint8_t)atoi(pp + 4);
            uint8_t action = (uint8_t)atoi(ap + 4);
            if (pump >= 1 && pump <= 4) {
                gw_sendPumpCommand(pump, action);
            }
        }
    } else if (strstr(json, "\"cmd\":\"cfg_get\"")) {
        gw_sendConfigGet();
    } else if (strstr(json, "\"cmd\":\"stats_get\"")) {
        gw_sendStatsGet();
    } else if (strstr(json, "\"cmd\":\"keepalive\"")) {
        gw_sendConfigGet();
        gLastKeepalive_ms = millis();
    } else if (strstr(json, "\"cmd\":\"telemetry\"")) {
        pond_sendTelemetry();
    } else if (strstr(json, "\"cmd\":\"cfg_set\"")) {
        auto getU32 = [](const char *j, const char *key) -> uint32_t {
            const char *p = strstr(j, key);
            if (!p) return 0;
            p += strlen(key);
            while (*p == ':' || *p == ' ') ++p;
            return (uint32_t)atol(p);
        };
        auto getU8 = [](const char *j, const char *key) -> uint8_t {
            const char *p = strstr(j, key);
            if (!p) return 0;
            p += strlen(key);
            while (*p == ':' || *p == ' ') ++p;
            return (uint8_t)atoi(p);
        };

        NodeConfig c;
        memset(&c, 0, sizeof(c));
        c.pump_min_runtime_ms   = getU32(json, "\"pmr\"");
        c.pump_min_cooldown_ms  = getU32(json, "\"pmc\"");
        c.replenish_runon_ms    = getU32(json, "\"pro\"");
        c.telemetry_interval_ms = getU32(json, "\"ti\"");
        c.network_timeout_ms    = getU32(json, "\"nt\"");
        c.ack_timeout_ms        = getU32(json, "\"at\"");
        c.ack_max_retries       = getU8(json,  "\"amr\"");
        c.boot_auto_mode        = getU8(json,  "\"bam\"");

        Serial.printf("[WS] cfg_set  pmr=%lu pmc=%lu pro=%lu ti=%lu nt=%lu at=%lu amr=%u bam=%u\n",
                      (unsigned long)c.pump_min_runtime_ms, (unsigned long)c.pump_min_cooldown_ms,
                      (unsigned long)c.replenish_runon_ms,  (unsigned long)c.telemetry_interval_ms,
                      (unsigned long)c.network_timeout_ms,  (unsigned long)c.ack_timeout_ms,
                      (unsigned)c.ack_max_retries, (unsigned)c.boot_auto_mode);
        gw_sendConfigSet(c);
        bcastLog("inf", "CONFIG_SET sent — waiting for CONFIG_RESP to confirm...");
    }
}

// =============================================================================
// HELP PRINT
// =============================================================================

static void printHelp() {
    Serial.println();
    Serial.println("=== LoRa Test | DUAL MODE: Gateway 0x01 + Pond 0x03 simultaneous ===");
    Serial.printf( "=== Web UI: http://192.168.4.1  (SSID: LoRaTest-TTGO) ===\n");
    Serial.println("  h/?  — this help");
    Serial.println("  --- Gateway commands (sender=0x01) ---");
    Serial.println("  c    — MSG_CONFIG_GET");
    Serial.println("  s    — MSG_STATS_GET");
    Serial.println("  k    — keepalive ping");
    Serial.println("  1/!  — Pump 1 ON/OFF  (force-override)");
    Serial.println("  2/@  — Pump 2 ON/OFF");
    Serial.println("  3/#  — Pump 3 ON/OFF");
    Serial.println("  4/$  — Pond Pump ON/OFF");
    Serial.println("  --- Pond commands (sender=0x03) ---");
    Serial.println("  t    — send telemetry to tank");
    Serial.println("  (pond pump commands auto-ACKed when received as 0x03)");
    Serial.println("=================================================");
    Serial.println();
}

// =============================================================================
// WIFI + WEB SERVER INIT
// =============================================================================

static void setupWiFi() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("LoRaTest-TTGO", "loratest1");
    delay(200);
    IPAddress ip = WiFi.softAPIP();
    Serial.printf("[WIFI] AP started  SSID=LoRaTest-TTGO  IP=%s\n", ip.toString().c_str());
}

static void setupWebServer() {
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(200, "text/html", INDEX_HTML);
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

                bool forMe = (pkt->header.magic_word == MY_NETWORK_MAGIC) &&
                             (pkt->header.target_id == NODE_GATEWAY     ||
                              pkt->header.target_id == NODE_POND_REMOTE ||
                              pkt->header.target_id == NODE_BROADCAST);
                if (forMe) {
                    decodeAndPrint(pkt, rssi, snr);
                } else {
                    Serial.printf("[RX] Ignored — magic=0x%04X target=0x%02X\n",
                                  pkt->header.magic_word, pkt->header.target_id);
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

    // ── Periodic LoRa actions (suppressed during quiet window after any TX) ──────
    // The quiet window ensures we stay in RX long enough to receive the tank's ACK
    // and the tank's relay command to the pond node (and then ACK that too).
    if (now >= gQuietUntil_ms) {
        if ((now - gLastKeepalive_ms) >= GATEWAY_KEEPALIVE_MS) {
            gLastKeepalive_ms = now;
            Serial.println("[GW] Keepalive CONFIG_GET...");
            bcastLog("inf", "Keepalive CONFIG_GET...");
            gw_sendConfigGet();
        }
        if ((now - gLastTelemetry_ms) >= POND_TELEMETRY_MS) {
            pond_sendTelemetry();
        }
    }

    // ── Serial commands ───────────────────────────────────────────────────────
    if (Serial.available()) {
        char cmd = (char)Serial.read();
        switch (cmd) {
            case 'h': case '?': printHelp();                                        break;
            case 'c': Serial.println("[GW] CONFIG_GET");   gw_sendConfigGet();      break;
            case 's': Serial.println("[GW] STATS_GET");    gw_sendStatsGet();       break;
            case 'k': Serial.println("[GW] Keepalive");    gw_sendConfigGet(); gLastKeepalive_ms = now; break;
            case '1': Serial.println("[GW] Pump 1 ON");    gw_sendPumpCommand(1,1); break;
            case '!': Serial.println("[GW] Pump 1 OFF");   gw_sendPumpCommand(1,0); break;
            case '2': Serial.println("[GW] Pump 2 ON");    gw_sendPumpCommand(2,1); break;
            case '@': Serial.println("[GW] Pump 2 OFF");   gw_sendPumpCommand(2,0); break;
            case '3': Serial.println("[GW] Pump 3 ON");    gw_sendPumpCommand(3,1); break;
            case '#': Serial.println("[GW] Pump 3 OFF");   gw_sendPumpCommand(3,0); break;
            case '4': Serial.println("[GW] Pond Pump ON"); gw_sendPumpCommand(4,1); break;
            case '$': Serial.println("[GW] Pond Pump OFF");gw_sendPumpCommand(4,0); break;
            case 't': Serial.println("[POND] Telemetry");  pond_sendTelemetry();    break;
            case '\r': case '\n': break;
            default:
                Serial.printf("[?] Unknown command '%c' — press 'h' for help\n", cmd);
                break;
        }
    }
}

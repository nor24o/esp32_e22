// =============================================================================
// TEST DEVICE — LoRa Communication Tester
// Target hardware: TTGO LoRa32 V1.0 (ESP32 + SX1276 radio chip)
//
// WHAT THIS DOES
// ─────────────────────────────────────────────────────────────────────────────
// This firmware turns one TTGO board into a complete test rig for the tank
// controller system.  It simultaneously simulates TWO devices:
//
//   [GATEWAY]  NODE_GATEWAY (0x01)
//     - Sends pump commands to the tank controller (P1/P2/P3/Pond)
//     - Requests and displays tank config and stats
//
//   [POND]     NODE_POND_REMOTE (0x03)
//     - Receives pump ON/OFF commands from the tank controller (0x02)
//     - Records the commanded pump state
//     - Sends its own telemetry every 10 s to NODE_BROADCAST (0xFF)
//       so both the tank and the gateway can see it without a second packet
//
// There are no ACK packets in this system.
// The pond confirms commands through its regular telemetry.
// The tank watches for that telemetry within a response window.
//
// HARDWARE — TTGO LoRa32 V1.0 SX1276 WIRING
// ─────────────────────────────────────────────────────────────────────────────
//   SCK=5  MISO=19  MOSI=27  NSS=18  RST=14  DIO0=26
//
// RF SETTINGS (must match tank firmware exactly)
//   868.0 MHz | BW=125 kHz | SF=9 | CR=4/5 | Sync=0x12 | CRC on
//
// WEB INTERFACE
// ─────────────────────────────────────────────────────────────────────────────
//   Connect your phone or laptop to Wi-Fi:
//     SSID: LoRaTest-TTGO
//     Password: loratest1
//   Then open a browser:  http://192.168.4.1
//
// SERIAL COMMANDS (115200 baud)
// ─────────────────────────────────────────────────────────────────────────────
//   h / ?    — print this help
//   c        — ask tank for its config  (CONFIG_GET)
//   s        — ask tank for its stats   (STATS_GET)
//   1 / !    — tank Pump 1 ON / OFF
//   2 / @    — tank Pump 2 ON / OFF
//   3 / #    — tank Pump 3 ON / OFF
//   4 / $    — tank Pond Pump ON / OFF
//   t        — send pond telemetry now  (also sent automatically every 10 s)
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "web_page.h"
#include "protocol.hpp"   // shared packet definitions — must match tank firmware
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// =============================================================================
// TTGO V1 PIN DEFINITIONS
// =============================================================================

#define LORA_NSS   18
#define LORA_DIO0  26   // SX1276 uses DIO0 (not DIO1) for RxDone/TxDone
#define LORA_NRST  14
#define LORA_DIO1  -1   // Not used on this board
#define LORA_SCK    5
#define LORA_MISO  19
#define LORA_MOSI  27

// The TTGO's SX1276 maxes out at 14 dBm.
// The tank's SX1262 uses 22 dBm — this is fine, they just need to hear each other.
#define LORA_TX_POWER  14

// =============================================================================
// TIMING CONSTANTS
// =============================================================================

// How often the gateway side sends a CONFIG_GET to the tank (acts as a heartbeat).
#define GATEWAY_KEEPALIVE_MS   30000UL

// How often the pond side sends its telemetry.
// Must match what the tank expects (same as tank's telemetry_interval_ms default).
#define POND_TELEMETRY_MS      10000UL

// How often to push a status JSON to the web dashboard.
#define STATUS_BCAST_MS         5000UL

// =============================================================================
// RADIO SETUP
// =============================================================================

static SPIClass        loraSPI(VSPI);
static SX1276          radio = new Module(LORA_NSS, LORA_DIO0, LORA_NRST, LORA_DIO1, loraSPI);
static volatile bool   rxFlag = false;

// Called from the DIO0 interrupt when a packet finishes receiving or transmitting.
void IRAM_ATTR onRxDone() { rxFlag = true; }

// =============================================================================
// WEB SERVER AND WEBSOCKET
// =============================================================================

static AsyncWebServer  server(80);
static AsyncWebSocket  ws("/ws");

static char gJsonBuf[600];   // scratch buffer for building JSON strings

// Commands arriving from the web browser are written here by the WS callback
// and read from the main loop().  The volatile flag signals "new command ready".
static char          gWsCmd[256];
static volatile bool gWsCmdReady = false;

static void broadcastJson(const char *json) {
    ws.textAll(json);
}

// =============================================================================
// STATE VARIABLES
// =============================================================================

static uint8_t  gMsgId    = 0;     // incremented with every packet we send
static int8_t   gLastRssi = 0;     // RSSI of the most recently received packet
static int8_t   gLastSnr  = 0;     // SNR of the most recently received packet

// Pond simulation state — updated when the tank sends us a pump command.
static bool gPondPumpOn = false;

// Timers for periodic actions.
static uint32_t gLastKeepalive_ms = 0;
static uint32_t gLastTelemetry_ms = 0;
static uint32_t gLastStatusBcast  = 0;

// =============================================================================
// JSON HELPERS — build and broadcast messages to the web dashboard
// =============================================================================

static void wsLog(const char *level, const char *msg) {
    // level: "rx" "tx" "err" "inf"
    snprintf(gJsonBuf, sizeof(gJsonBuf),
             "{\"t\":\"log\",\"lvl\":\"%s\",\"msg\":\"%s\"}", level, msg);
    broadcastJson(gJsonBuf);
}

static void wsStatus() {
    snprintf(gJsonBuf, sizeof(gJsonBuf),
             "{\"t\":\"status\",\"heap\":%lu,\"uptime\":%lu,\"clients\":%u}",
             (unsigned long)ESP.getFreeHeap(),
             (unsigned long)(millis() / 1000),
             (unsigned)ws.count());
    broadcastJson(gJsonBuf);
}

static void wsTx(uint8_t msgType, uint8_t targetId, uint8_t msgId) {
    snprintf(gJsonBuf, sizeof(gJsonBuf),
             "{\"t\":\"tx\",\"mtype\":%u,\"target\":%u,\"id\":%u}",
             msgType, targetId, msgId);
    broadcastJson(gJsonBuf);
}

static void wsTelemetry(const TelemetryData &t, int8_t rssi, int8_t snr) {
    snprintf(gJsonBuf, sizeof(gJsonBuf),
             "{\"t\":\"telem\","
             "\"wl\":%u,\"auto\":%u,\"p1\":%u,\"p2\":%u,\"p3\":%u,\"pond\":%u,"
             "\"tmp\":%d,\"hum\":%u,\"err\":%u,"
             "\"nrssi\":%d,\"nsnr\":%d,\"rssi\":%d,\"snr\":%d}",
             t.water_level,
             (t.system_flags & FLAG_AUTO_MODE) ? 1 : 0,
             (t.system_flags & FLAG_PUMP_P1)   ? 1 : 0,
             (t.system_flags & FLAG_PUMP_P2)   ? 1 : 0,
             (t.system_flags & FLAG_PUMP_P3)   ? 1 : 0,
             (t.system_flags & FLAG_PUMP_POND)  ? 1 : 0,
             (int)t.temperature, (unsigned)t.humidity, (unsigned)t.error_code,
             (int)t.last_rssi, (int)t.last_snr,
             (int)rssi, (int)snr);
    broadcastJson(gJsonBuf);
}

static void wsConfig(const NodeConfig &c, int8_t rssi, int8_t snr) {
    snprintf(gJsonBuf, sizeof(gJsonBuf),
             "{\"t\":\"cfg\","
             "\"pmr\":%lu,\"pmc\":%lu,\"pro\":%lu,\"ti\":%lu,\"nt\":%lu,\"crt\":%lu,"
             "\"bam\":%u,"
             "\"rssi\":%d,\"snr\":%d}",
             (unsigned long)c.pump_min_runtime_ms,
             (unsigned long)c.pump_min_cooldown_ms,
             (unsigned long)c.replenish_runon_ms,
             (unsigned long)c.telemetry_interval_ms,
             (unsigned long)c.network_timeout_ms,
             (unsigned long)c.cmd_response_timeout_ms,
             (unsigned)c.boot_auto_mode,
             (int)rssi, (int)snr);
    broadcastJson(gJsonBuf);
}

static void wsStats(const StatsPayload &s, int8_t rssi, int8_t snr) {
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

static void wsCommand(uint8_t pump, uint8_t action, uint8_t msgId, int8_t rssi, int8_t snr) {
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
        Serial.printf("[WS] Client #%u connected from %s\n",
                      client->id(), client->remoteIP().toString().c_str());
        wsStatus();

    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WS] Client #%u disconnected\n", client->id());

    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        // Only handle single complete text frames.  Ignore binary, partial, or fragmented.
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
// PACKET TRANSMIT HELPER
// =============================================================================

// Fills in the packet header, transmits the packet, and returns to receive mode.
static void sendPacket(LoRaPacket &pkt, uint8_t target, uint8_t msgType, uint8_t senderId) {
    pkt.header.magic_word = MY_NETWORK_MAGIC;
    pkt.header.target_id  = target;
    pkt.header.sender_id  = senderId;
    pkt.header.msg_id     = ++gMsgId;
    pkt.header.msg_type   = msgType;

    int state = radio.transmit((uint8_t *)&pkt, sizeof(LoRaPacket));

    if (state == RADIOLIB_ERR_NONE) {
        Serial.printf("[TX] type=%-2u  to=0x%02X  from=0x%02X  id=%-3u  OK\n",
                      msgType, target, senderId, gMsgId);
        wsTx(msgType, target, gMsgId);
    } else {
        Serial.printf("[TX] FAILED — RadioLib error %d\n", state);
        wsLog("err", "TX failed");
    }

    radio.startReceive();
}

// =============================================================================
// GATEWAY SIDE — actions the gateway controller takes
// =============================================================================

// Ask the tank to send back its current configuration.
static void gw_requestConfig() {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    sendPacket(pkt, NODE_TANK_LOCAL, MSG_CONFIG_GET, NODE_GATEWAY);
}

// Ask the tank to send back its operational statistics.
static void gw_requestStats() {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    sendPacket(pkt, NODE_TANK_LOCAL, MSG_STATS_GET, NODE_GATEWAY);
}

// Tell the tank to turn a pump ON or OFF.
// pump_id: 1=P1, 2=P2, 3=P3, 4=Pond pump
// action:  1=ON, 0=OFF
// Gateway commands are always obeyed by the tank — no force flag needed.
static void gw_sendPumpCommand(uint8_t pump_id, uint8_t action) {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.payload.command.target_pump = pump_id;
    pkt.payload.command.action      = action;
    pkt.payload.command.reserved    = 0;
    sendPacket(pkt, NODE_TANK_LOCAL, MSG_COMMAND, NODE_GATEWAY);
}

// Push a new configuration to the tank.
static void gw_sendConfigSet(const NodeConfig &cfg) {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.payload.config = cfg;
    sendPacket(pkt, NODE_TANK_LOCAL, MSG_CONFIG_SET, NODE_GATEWAY);
}

// =============================================================================
// POND SIDE — actions the pond controller takes
// =============================================================================

// Send the pond's current state to the network.
// We use NODE_BROADCAST (0xFF) so both the tank AND the gateway receive the
// same packet without needing to send it twice.
static void pond_sendTelemetry() {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.payload.telemetry.water_level  = 2;      // pond always has water (constant for test)
    pkt.payload.telemetry.system_flags = gPondPumpOn ? FLAG_PUMP_POND : 0;
    pkt.payload.telemetry.temperature  = 210;    // 21.0 °C placeholder
    pkt.payload.telemetry.humidity     = 65;
    pkt.payload.telemetry.error_code   = 0;
    pkt.payload.telemetry.last_rssi    = gLastRssi;
    pkt.payload.telemetry.last_snr     = gLastSnr;

    sendPacket(pkt, NODE_BROADCAST, MSG_TELEMETRY, NODE_POND_REMOTE);
    gLastTelemetry_ms = millis();

    Serial.printf("[POND] Telemetry broadcast — pond pump is %s\n",
                  gPondPumpOn ? "ON" : "OFF");
}

// =============================================================================
// INCOMING PACKET HANDLER
// =============================================================================

// Called for every valid packet we receive.  Prints a detailed log and
// pushes the data to the web dashboard via WebSocket.
static void handleIncomingPacket(const LoRaPacket *pkt, int8_t rssi, int8_t snr) {
    gLastRssi = rssi;
    gLastSnr  = snr;

    Serial.println("┌──────────────────────────────────────────────────");
    Serial.printf( "│ RX  from=0x%02X  type=%u  id=%u  RSSI=%d dBm  SNR=%d dB\n",
                   pkt->header.sender_id, pkt->header.msg_type,
                   pkt->header.msg_id, rssi, snr);

    switch (pkt->header.msg_type) {

        // ── Telemetry from the tank controller ───────────────────────────────
        case MSG_TELEMETRY: {
            const TelemetryData &t = pkt->payload.telemetry;
            const char *errStr =
                t.error_code == 0 ? "OK" :
                t.error_code == 3 ? "NO-COMMS" : "UNKNOWN";

            Serial.println("│ MSG_TELEMETRY");
            Serial.printf( "│   Sender       : 0x%02X\n",  pkt->header.sender_id);
            Serial.printf( "│   Water level  : %u / 3\n",  t.water_level);
            Serial.printf( "│   Mode         : %s\n",      (t.system_flags & FLAG_AUTO_MODE) ? "AUTO" : "MANUAL");
            Serial.printf( "│   Pumps        : P1=%s  P2=%s  P3=%s  Pond=%s\n",
                           (t.system_flags & FLAG_PUMP_P1)   ? "ON" : "off",
                           (t.system_flags & FLAG_PUMP_P2)   ? "ON" : "off",
                           (t.system_flags & FLAG_PUMP_P3)   ? "ON" : "off",
                           (t.system_flags & FLAG_PUMP_POND)  ? "ON" : "off");
            Serial.printf( "│   Temperature  : %.1f °C   Humidity: %u%%\n",
                           t.temperature / 10.0f, t.humidity);
            Serial.printf( "│   Error        : %s (%u)\n", errStr, t.error_code);
            Serial.printf( "│   Node signal  : RSSI=%d dBm  SNR=%d dB\n",
                           t.last_rssi, t.last_snr);
            wsTelemetry(t, rssi, snr);
            break;
        }

        // ── Pump command to the pond ──────────────────────────────────────────
        // The tank sends this when it wants the pond pump to change state.
        // We apply the commanded state and will report it in the next telemetry.
        case MSG_COMMAND: {
            const CommandData &cmd = pkt->payload.command;
            Serial.println("│ MSG_COMMAND");
            Serial.printf( "│   Pump         : %u\n",  cmd.target_pump);
            Serial.printf( "│   Action       : %s\n",  cmd.action ? "ON" : "OFF");
            wsCommand(cmd.target_pump, cmd.action, pkt->header.msg_id, rssi, snr);

            // If this command is addressed to us as the pond node, apply it.
            // The tank will see the updated state in our next telemetry broadcast.
            if (pkt->header.target_id == NODE_POND_REMOTE && cmd.target_pump == 1) {
                gPondPumpOn = (cmd.action == 1);
                Serial.printf("│   → Pond: pump commanded %s\n",
                              gPondPumpOn ? "ON" : "OFF");
                Serial.println("│     (will confirm in next telemetry broadcast)");
            }
            break;
        }

        // ── Tank's current configuration ──────────────────────────────────────
        case MSG_CONFIG_RESP: {
            const NodeConfig &c = pkt->payload.config;
            Serial.println("│ MSG_CONFIG_RESP");
            Serial.printf( "│   pump_min_runtime_ms       = %lu ms\n",  c.pump_min_runtime_ms);
            Serial.printf( "│   pump_min_cooldown_ms      = %lu ms\n",  c.pump_min_cooldown_ms);
            Serial.printf( "│   replenish_runon_ms        = %lu ms\n",  c.replenish_runon_ms);
            Serial.printf( "│   telemetry_interval_ms     = %lu ms\n",  c.telemetry_interval_ms);
            Serial.printf( "│   network_timeout_ms        = %lu ms\n",  c.network_timeout_ms);
            Serial.printf( "│   cmd_response_timeout_ms   = %lu ms\n",  c.cmd_response_timeout_ms);
            Serial.printf( "│   boot_auto_mode            = %u\n",      c.boot_auto_mode);
            wsConfig(c, rssi, snr);
            break;
        }

        // ── Tank's operational statistics ─────────────────────────────────────
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
            wsStats(s, rssi, snr);
            break;
        }

        default:
            Serial.printf("│ Unknown msg_type=%u — ignored\n", pkt->header.msg_type);
            break;
    }

    Serial.println("└──────────────────────────────────────────────────");
}

// =============================================================================
// WEBSOCKET COMMAND PROCESSOR
// =============================================================================

// The web dashboard sends JSON commands like {"cmd":"pump","p":1,"a":1}
// We parse them here and call the appropriate action function.
static void processWebCommand(const char *json) {
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
        gw_requestConfig();

    } else if (strstr(json, "\"cmd\":\"stats_get\"")) {
        gw_requestStats();

    } else if (strstr(json, "\"cmd\":\"keepalive\"")) {
        gw_requestConfig();
        gLastKeepalive_ms = millis();

    } else if (strstr(json, "\"cmd\":\"telemetry\"")) {
        pond_sendTelemetry();

    } else if (strstr(json, "\"cmd\":\"cfg_set\"")) {
        // Helper to extract an unsigned 32-bit integer from a JSON key like "\"pmr\":30000"
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
        c.pump_min_runtime_ms     = getU32(json, "\"pmr\"");
        c.pump_min_cooldown_ms    = getU32(json, "\"pmc\"");
        c.replenish_runon_ms      = getU32(json, "\"pro\"");
        c.telemetry_interval_ms   = getU32(json, "\"ti\"");
        c.network_timeout_ms      = getU32(json, "\"nt\"");
        c.cmd_response_timeout_ms = getU32(json, "\"crt\"");
        c.boot_auto_mode          = getU8(json,  "\"bam\"");

        gw_sendConfigSet(c);
        wsLog("inf", "CONFIG_SET sent — waiting for CONFIG_RESP to confirm...");
    }
}

// =============================================================================
// SERIAL HELP TEXT
// =============================================================================

static void printHelp() {
    Serial.println();
    Serial.println("=== LoRa Test Device — Gateway (0x01) + Pond (0x03) ===");
    Serial.printf( "=== Web UI: http://192.168.4.1  (SSID: LoRaTest-TTGO) ===\n");
    Serial.println("  h / ?  — this help");
    Serial.println("  c      — ask tank for config  (MSG_CONFIG_GET)");
    Serial.println("  s      — ask tank for stats   (MSG_STATS_GET)");
    Serial.println("  1 / !  — tank Pump 1 ON / OFF");
    Serial.println("  2 / @  — tank Pump 2 ON / OFF");
    Serial.println("  3 / #  — tank Pump 3 ON / OFF");
    Serial.println("  4 / $  — tank Pond Pump ON / OFF");
    Serial.println("  t      — send pond telemetry broadcast now");
    Serial.println("=======================================================");
    Serial.println();
}

// =============================================================================
// WIFI + WEB SERVER SETUP
// =============================================================================

static void setupWiFi() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("LoRaTest-TTGO", "loratest1");
    delay(200);
    Serial.printf("[WIFI] AP started — SSID: LoRaTest-TTGO  IP: %s\n",
                  WiFi.softAPIP().toString().c_str());
}

static void setupWebServer() {
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(200, "text/html", INDEX_HTML);
    });

    server.begin();
    Serial.println("[HTTP] Web server running on port 80");
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n============================================================");
    Serial.println("[BOOT] LoRa Test Device — TTGO LoRa32 V1.0 (SX1276)");
    Serial.printf( "[BOOT] Simulating: Gateway (0x01) + Pond (0x03)\n");
    Serial.printf( "[BOOT] RF: %.1f MHz  BW=%.0f kHz  SF%d  CR=4/%d  Sync=0x%02X  PWR=%d dBm\n",
                   LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SF, LORA_CR,
                   LORA_SYNC_WORD, LORA_TX_POWER);
    Serial.println("------------------------------------------------------------");

    setupWiFi();
    setupWebServer();

    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    int state = radio.begin(
        LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SF,
        LORA_CR, LORA_SYNC_WORD, LORA_TX_POWER, LORA_PREAMBLE);

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] radio.begin() failed — code %d\n", state);
        Serial.println("[ERROR] Check: SPI wiring (SCK/MISO/MOSI/NSS), RST, DIO0, power.");
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

    Serial.printf("[BOOT] SX1276 OK — packet size: %u bytes\n",
                  (unsigned)sizeof(LoRaPacket));
    Serial.println("============================================================");

    gLastKeepalive_ms = millis();
    gLastTelemetry_ms = millis();
    gLastStatusBcast  = millis();

    printHelp();
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
    uint32_t now = millis();

    // ── Web dashboard commands ────────────────────────────────────────────────
    if (gWsCmdReady) {
        gWsCmdReady = false;
        Serial.printf("[WS] Command: %s\n", gWsCmd);
        processWebCommand(gWsCmd);
    }

    ws.cleanupClients();

    // ── Periodic web dashboard status push ───────────────────────────────────
    if ((now - gLastStatusBcast) >= STATUS_BCAST_MS) {
        gLastStatusBcast = now;
        wsStatus();
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

                // Accept packets addressed to us as gateway, as pond, or broadcast.
                bool isOurNetwork  = (pkt->header.magic_word == MY_NETWORK_MAGIC);
                bool addressedToUs = (pkt->header.target_id == NODE_GATEWAY     ||
                                      pkt->header.target_id == NODE_POND_REMOTE ||
                                      pkt->header.target_id == NODE_BROADCAST);

                if (isOurNetwork && addressedToUs) {
                    handleIncomingPacket(pkt, rssi, snr);
                } else {
                    Serial.printf("[RX] Ignored — magic=0x%04X  target=0x%02X\n",
                                  pkt->header.magic_word, pkt->header.target_id);
                }
            } else {
                Serial.printf("[RX] Read error: %d\n", state);
                wsLog("err", "RX read error");
            }

        } else if (rxLen > 0) {
            // Wrong size — discard without reading into our fixed buffer.
            uint8_t discard[255];
            radio.readData(discard, (size_t)min(rxLen, 255));
            Serial.printf("[RX] Ignored — wrong length %d (expected %u)\n",
                          rxLen, (unsigned)sizeof(LoRaPacket));
        }

        radio.startReceive();
    }

    // ── Periodic gateway keepalive to the tank ────────────────────────────────
    // Asking for config serves as a heartbeat — the tank uses the reception
    // timestamp to know the gateway is still alive.
    if ((now - gLastKeepalive_ms) >= GATEWAY_KEEPALIVE_MS) {
        gLastKeepalive_ms = now;
        Serial.println("[GW] Sending keepalive (CONFIG_GET)");
        wsLog("inf", "Keepalive CONFIG_GET...");
        gw_requestConfig();
    }

    // ── Periodic pond telemetry broadcast ────────────────────────────────────
    // Sent to NODE_BROADCAST so both the tank and the gateway receive it.
    if ((now - gLastTelemetry_ms) >= POND_TELEMETRY_MS) {
        pond_sendTelemetry();
    }

    // ── Serial commands ───────────────────────────────────────────────────────
    if (Serial.available()) {
        char cmd = (char)Serial.read();
        switch (cmd) {
            case 'h': case '?':  printHelp();                          break;
            case 'c':  Serial.println("[GW] Requesting config");  gw_requestConfig();       break;
            case 's':  Serial.println("[GW] Requesting stats");   gw_requestStats();        break;
            case '1':  Serial.println("[GW] Pump 1 → ON");        gw_sendPumpCommand(1,1);  break;
            case '!':  Serial.println("[GW] Pump 1 → OFF");       gw_sendPumpCommand(1,0);  break;
            case '2':  Serial.println("[GW] Pump 2 → ON");        gw_sendPumpCommand(2,1);  break;
            case '@':  Serial.println("[GW] Pump 2 → OFF");       gw_sendPumpCommand(2,0);  break;
            case '3':  Serial.println("[GW] Pump 3 → ON");        gw_sendPumpCommand(3,1);  break;
            case '#':  Serial.println("[GW] Pump 3 → OFF");       gw_sendPumpCommand(3,0);  break;
            case '4':  Serial.println("[GW] Pond pump → ON");     gw_sendPumpCommand(4,1);  break;
            case '$':  Serial.println("[GW] Pond pump → OFF");    gw_sendPumpCommand(4,0);  break;
            case 't':  Serial.println("[POND] Sending telemetry");pond_sendTelemetry();      break;
            case '\r': case '\n': break;
            default:
                Serial.printf("[?] Unknown key '%c' — press 'h' for help\n", cmd);
                break;
        }
    }
}

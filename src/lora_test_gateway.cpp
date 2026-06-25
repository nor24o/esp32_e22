// =============================================================================
// LORA COMMUNICATION TEST — Gateway Simulator
// Target  : TTGO LoRa32 V1.0 (ESP32 + SX1276 HPD13A module)
// Role    : NODE_GATEWAY (0x01) — sends commands, receives telemetry/responses
//           from Tank Controller Node 0x02 running the main firmware.
//
// TTGO V1 LoRa pin map (SX1276):
//   SCK=5  MISO=19  MOSI=27  NSS=18  RST=14  DIO0=26
//
// RF settings must match the tank node exactly:
//   868.0 MHz | BW 125 kHz | SF9 | CR 4/5 | Sync 0x12 | CRC on
//
// Serial commands (115200 baud):
//   c  — send MSG_CONFIG_GET  (read tank config)
//   s  — send MSG_STATS_GET   (read tank stats)
//   1  — pump 1 ON  (force-override)
//   !  — pump 1 OFF (force-override)
//   2  — pump 2 ON
//   @  — pump 2 OFF
//   3  — pump 3 ON
//   #  — pump 3 OFF
//   h  — print this help
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <stdint.h>
#include <string.h>

// =============================================================================
// TTGO V1 PIN MAP (SX1276)
// =============================================================================

#define LORA_NSS   18
#define LORA_DIO0  26   // SX1276 uses DIO0 for RxDone/TxDone (not DIO1 like SX1262)
#define LORA_NRST  14
#define LORA_DIO1  -1   // not connected / not needed for basic RX
#define LORA_SCK    5
#define LORA_MISO  19
#define LORA_MOSI  27

// =============================================================================
// RF SETTINGS — must match tank node exactly
// =============================================================================

#define LORA_FREQUENCY   868.0f
#define LORA_BANDWIDTH   125.0f
#define LORA_SF          9
#define LORA_CR          5
#define LORA_SYNC_WORD   0x12
#define LORA_TX_POWER    17     // TTGO V1 max safe output
#define LORA_PREAMBLE    8

// =============================================================================
// PROTOCOL — identical to tank firmware (keep in sync with protocol.hpp)
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
};  // 6 bytes

struct __attribute__((packed)) TelemetryData {
    uint8_t  water_level;
    uint8_t  system_flags;  // bit0=Auto bit1=P1 bit2=P2 bit3=P3 bit4=PondP
    int16_t  temperature;   // °C × 10
    uint8_t  humidity;
    uint8_t  error_code;    // 0=OK 1=OC 2=DryRun 3=Comms
    int8_t   last_rssi;
    int8_t   last_snr;
};  // 8 bytes

struct __attribute__((packed)) CommandData {
    uint8_t target_pump;    // 1-4; also used as acked-msg-id in ACK frames
    uint8_t action;         // 0=Off 1=On
    uint8_t flags;          // bit0=Force Override
};  // 3 bytes

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
};  // 32 bytes

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
};  // 32 bytes

struct __attribute__((packed)) LoRaPacket {
    LoRaHeader header;
    union {
        TelemetryData telemetry;
        CommandData   command;
        NodeConfig    config;
        StatsPayload  stats;
    } payload;
};  // 38 bytes

static_assert(sizeof(LoRaPacket) <= 255, "LoRaPacket exceeds max RadioLib payload");

// =============================================================================
// RADIO INSTANCE
// =============================================================================

static SPIClass loraSPI(VSPI);
// SX1276: Module(NSS, DIO0, RESET, DIO1)
static SX1276 radio = new Module(LORA_NSS, LORA_DIO0, LORA_NRST, LORA_DIO1, loraSPI);

static volatile bool rxFlag = false;
static uint8_t       gMsgId = 0;

void IRAM_ATTR onRxDone() { rxFlag = true; }

// =============================================================================
// HELPERS
// =============================================================================

static void printHelp() {
    Serial.println();
    Serial.println("=== LoRa Gateway Test — Commands ===");
    Serial.println("  c   — MSG_CONFIG_GET  (read tank config)");
    Serial.println("  s   — MSG_STATS_GET   (read tank stats)");
    Serial.println("  1   — Pump 1 ON  (force-override)");
    Serial.println("  !   — Pump 1 OFF");
    Serial.println("  2   — Pump 2 ON");
    Serial.println("  @   — Pump 2 OFF");
    Serial.println("  3   — Pump 3 ON");
    Serial.println("  #   — Pump 3 OFF");
    Serial.println("  h   — this help");
    Serial.println("=====================================");
    Serial.println("Listening for telemetry from node 0x02...");
    Serial.println();
}

static void sendPacket(LoRaPacket &pkt) {
    pkt.header.magic_word = MY_NETWORK_MAGIC;
    pkt.header.sender_id  = NODE_GATEWAY;
    pkt.header.msg_id     = ++gMsgId;

    int state = radio.transmit((uint8_t *)&pkt, sizeof(LoRaPacket));
    if (state == RADIOLIB_ERR_NONE) {
        Serial.printf("[TX] type=%u  msg_id=%u  target=0x%02X  OK\n",
                      pkt.header.msg_type, pkt.header.msg_id, pkt.header.target_id);
    } else {
        Serial.printf("[TX] FAILED  state=%d\n", state);
    }
    radio.startReceive();
}

static void sendConfigGet() {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.header.target_id = NODE_TANK_LOCAL;
    pkt.header.msg_type  = MSG_CONFIG_GET;
    sendPacket(pkt);
}

static void sendStatsGet() {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.header.target_id = NODE_TANK_LOCAL;
    pkt.header.msg_type  = MSG_STATS_GET;
    sendPacket(pkt);
}

static void sendPumpCommand(uint8_t pump, uint8_t action) {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.header.target_id         = NODE_TANK_LOCAL;
    pkt.header.msg_type          = MSG_COMMAND;
    pkt.payload.command.target_pump = pump;
    pkt.payload.command.action      = action;
    pkt.payload.command.flags       = 0x01;  // force-override auto mode
    sendPacket(pkt);
}

// =============================================================================
// PACKET DECODER
// =============================================================================

static void decodeAndPrint(const LoRaPacket *pkt, int8_t rssi, int8_t snr) {
    Serial.println("┌─────────────────────────────────────────");
    Serial.printf( "│ RX  from=0x%02X  msg_id=%u  RSSI=%d dBm  SNR=%d dB\n",
                   pkt->header.sender_id, pkt->header.msg_id, rssi, snr);

    switch (pkt->header.msg_type) {
        case MSG_TELEMETRY: {
            const TelemetryData &t = pkt->payload.telemetry;
            const char *errStr =
                t.error_code == 0 ? "OK"          :
                t.error_code == 1 ? "OVERCURRENT"  :
                t.error_code == 2 ? "DRY-RUN"      :
                t.error_code == 3 ? "NO-COMMS"     : "UNKNOWN";
            Serial.println("│ MSG_TELEMETRY");
            Serial.printf( "│   Water level : %u/3\n",     t.water_level);
            Serial.printf( "│   Mode        : %s\n",       (t.system_flags & 0x01) ? "AUTO" : "MANUAL");
            Serial.printf( "│   Pumps       : P1=%s P2=%s P3=%s Pond=%s\n",
                           (t.system_flags & 0x02) ? "ON" : "off",
                           (t.system_flags & 0x04) ? "ON" : "off",
                           (t.system_flags & 0x08) ? "ON" : "off",
                           (t.system_flags & 0x10) ? "ON" : "off");
            Serial.printf( "│   Temp        : %.1f °C\n",  t.temperature / 10.0f);
            Serial.printf( "│   Humidity    : %u %%\n",    t.humidity);
            Serial.printf( "│   Error       : %s (%u)\n",  errStr, t.error_code);
            Serial.printf( "│   Node RSSI   : %d dBm  SNR: %d dB (last RX at node)\n",
                           t.last_rssi, t.last_snr);
            break;
        }
        case MSG_ACK: {
            Serial.println("│ MSG_ACK");
            Serial.printf( "│   Acked msg_id: %u\n", pkt->payload.command.target_pump);
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
            Serial.printf( "│   overcurrent_thresh  : %u (raw ADC)\n", c.overcurrent_thresh);
            Serial.printf( "│   dryrun_thresh       : %u (raw ADC)\n", c.dryrun_thresh);
            Serial.printf( "│   ack_max_retries     : %u\n",      c.ack_max_retries);
            Serial.printf( "│   boot_auto_mode      : %u\n",      c.boot_auto_mode);
            Serial.printf( "│   oc_grace_ticks      : %u (× 50 ms)\n", c.overcurrent_grace_ticks);
            Serial.printf( "│   fault_lockout       : %s\n",      c.fault_lockout_enabled ? "ENABLED" : "warn-only");
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
            break;
        }
        default:
            Serial.printf("│ Unknown msg_type=%u\n", pkt->header.msg_type);
            break;
    }
    Serial.println("└─────────────────────────────────────────");
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n============================================================");
    Serial.println("[BOOT] LoRa Gateway Test — TTGO LoRa32 V1.0 (SX1276)");
    Serial.printf( "[INIT] %.1f MHz  BW=%.0f kHz  SF%d  CR4/%d  Sync=0x%02X\n",
                   LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SF, LORA_CR, LORA_SYNC_WORD);
    Serial.printf( "[INIT] Pins: SCK=%d MISO=%d MOSI=%d NSS=%d RST=%d DIO0=%d\n",
                   LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS, LORA_NRST, LORA_DIO0);
    Serial.println("------------------------------------------------------------");

    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    int state = radio.begin(
        LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SF,
        LORA_CR, LORA_SYNC_WORD, LORA_TX_POWER, LORA_PREAMBLE);

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] radio.begin() failed — code %d\n", state);
        Serial.println("[ERROR] Check: SX1276 power, SPI wiring, NSS/RST/DIO0 pins.");
        Serial.println("[ERROR] Halting.");
        while (true) { delay(1000); }
    }

    radio.setCRC(true);
    radio.setPacketReceivedAction(onRxDone);

    state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] startReceive() failed — code %d\n", state);
        while (true) { delay(1000); }
    }

    Serial.printf("[BOOT] SX1276 OK  |  packet size: %u bytes\n", (unsigned)sizeof(LoRaPacket));
    Serial.println("============================================================");

    printHelp();
}

// =============================================================================
// LOOP
// =============================================================================

void loop() {
    // ── Handle received packet ───────────────────────────────────────────────
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

                if (pkt->header.magic_word == MY_NETWORK_MAGIC &&
                    (pkt->header.target_id == NODE_GATEWAY ||
                     pkt->header.target_id == NODE_BROADCAST)) {
                    decodeAndPrint(pkt, rssi, snr);
                } else {
                    Serial.printf("[RX] Ignored — wrong magic (0x%04X) or target (0x%02X)\n",
                                  pkt->header.magic_word, pkt->header.target_id);
                }
            } else {
                Serial.printf("[RX] readData error: %d\n", state);
            }
        } else if (rxLen > 0) {
            // Wrong length — drain and discard
            uint8_t discard[255];
            radio.readData(discard, (size_t)min(rxLen, 255));
            Serial.printf("[RX] Ignored — unexpected length %d (expected %u)\n",
                          rxLen, (unsigned)sizeof(LoRaPacket));
        }

        radio.startReceive();
    }

    // ── Handle serial commands ───────────────────────────────────────────────
    if (Serial.available()) {
        char cmd = (char)Serial.read();
        switch (cmd) {
            case 'c': Serial.println("[CMD] Sending CONFIG_GET...");  sendConfigGet();           break;
            case 's': Serial.println("[CMD] Sending STATS_GET...");   sendStatsGet();            break;
            case '1': Serial.println("[CMD] Pump 1 ON  (override)");  sendPumpCommand(1, 1);    break;
            case '!': Serial.println("[CMD] Pump 1 OFF (override)");  sendPumpCommand(1, 0);    break;
            case '2': Serial.println("[CMD] Pump 2 ON  (override)");  sendPumpCommand(2, 1);    break;
            case '@': Serial.println("[CMD] Pump 2 OFF (override)");  sendPumpCommand(2, 0);    break;
            case '3': Serial.println("[CMD] Pump 3 ON  (override)");  sendPumpCommand(3, 1);    break;
            case '#': Serial.println("[CMD] Pump 3 OFF (override)");  sendPumpCommand(3, 0);    break;
            case 'h': case '?': printHelp();                                                     break;
            case '\r': case '\n': break;  // ignore line endings
            default:
                Serial.printf("[CMD] Unknown command '%c' — press 'h' for help\n", cmd);
                break;
        }
    }
}

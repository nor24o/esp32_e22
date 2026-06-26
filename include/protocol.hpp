#pragma once

#include <stdint.h>
#include <string.h>

// =============================================================================
// SHARED NETWORK PROTOCOL
// This file is included by ALL devices: tank controller, gateway, and pond.
// Change it here once and rebuild all devices together.
//
// NETWORK OVERVIEW
// ─────────────────────────────────────────────────────────────────────────────
//  Node IDs:
//    0x01 = Gateway Controller  (the "boss" — its commands are always obeyed)
//    0x02 = Tank Controller     (controls local pumps P1/P2/P3, relays to pond)
//    0x03 = Pond Controller     (controls the pond pump, reports state)
//    0xFF = Broadcast           (every device listens to this)
//
//  Communication flow:
//    Gateway → Tank  : commands (pump ON/OFF, config read/write)
//    Tank    → Pond  : pond pump ON/OFF command
//    Tank    → Gateway : telemetry every 10 s (water level, pump states, etc.)
//    Pond    → Broadcast : telemetry every 10 s (pump state, so BOTH tank and
//                          gateway can see it without a second transmission)
//
//  No ACK packets — devices confirm commands through their next telemetry.
//  The tank waits up to cmd_response_timeout_ms to see the pond's telemetry
//  show the expected pump state.  After that window it just logs a warning.
// =============================================================================

// ── LoRa radio settings ───────────────────────────────────────────────────────
// ALL devices must use identical values or they cannot hear each other.
// TX power is NOT here because it depends on the hardware (tank = 22 dBm, TTGO = 14 dBm).
#define LORA_FREQUENCY   868.0f   // MHz  — EU ISM band
#define LORA_BANDWIDTH   125.0f   // kHz
#define LORA_SF          9        // Spreading factor (higher = longer range, slower)
#define LORA_CR          5        // Coding rate (4/5)
#define LORA_SYNC_WORD   0x12     // Private network identifier — keeps us off public traffic
#define LORA_PREAMBLE    8        // Preamble symbols

// ── Node IDs ─────────────────────────────────────────────────────────────────
#define MY_NETWORK_MAGIC  0x5A6B  // Magic number in every packet to reject foreign traffic
#define NODE_GATEWAY      0x01
#define NODE_TANK_LOCAL   0x02
#define NODE_POND_REMOTE  0x03
#define NODE_BROADCAST    0xFF

// ── Message types ─────────────────────────────────────────────────────────────
// Each packet carries one of these types in its header.
enum MessageType : uint8_t {
    MSG_TELEMETRY   = 1,   // Periodic status report (tank → gateway, pond → broadcast)
    MSG_COMMAND     = 2,   // Turn a pump ON or OFF
    MSG_CONFIG_GET  = 4,   // Gateway asks tank to send back its current config
    MSG_CONFIG_SET  = 5,   // Gateway writes a new config to the tank
    MSG_CONFIG_RESP = 6,   // Tank replies with its current config (after GET or SET)
    MSG_STATS_GET   = 7,   // Gateway asks tank for operational stats
    MSG_STATS_RESP  = 8,   // Tank replies with stats
};

// ── Packet structures ─────────────────────────────────────────────────────────
// __attribute__((packed)) stops the compiler from adding padding bytes so the
// struct size is exactly what the field list says.  Important for radio packets.

// Every packet starts with this 6-byte header.
struct __attribute__((packed)) LoRaHeader {
    uint16_t magic_word;  // Must equal MY_NETWORK_MAGIC or the packet is discarded
    uint8_t  target_id;   // Who should process this packet
    uint8_t  sender_id;   // Who sent it
    uint8_t  msg_id;      // Counts up 0-255 and wraps; used to detect duplicate packets
    uint8_t  msg_type;    // One of the MessageType values above
};  // 6 bytes

// Telemetry payload — carried in MSG_TELEMETRY packets.
// Tank sends this to gateway every 10 s.
// Pond sends this to broadcast (0xFF) every 10 s.
struct __attribute__((packed)) TelemetryData {
    uint8_t  water_level;   // 0 = empty, 1/2/3 = number of active float switches
    uint8_t  system_flags;  // Bit flags — see below
    int16_t  temperature;   // Temperature in °C × 10  (e.g. 215 = 21.5 °C)
    uint8_t  humidity;      // Relative humidity 0–100 %
    uint8_t  error_code;    // 0 = OK,  3 = no comms with a peer
    int8_t   last_rssi;     // Signal strength of the last packet we received (dBm)
    int8_t   last_snr;      // Signal-to-noise ratio of the last received packet (dB)
};  // 8 bytes

// system_flags bit positions in TelemetryData:
//   bit 0 = auto mode active (1) or manual mode (0)
//   bit 1 = pump P1 is running
//   bit 2 = pump P2 is running
//   bit 3 = pump P3 is running
//   bit 4 = pond pump is running
#define FLAG_AUTO_MODE  (1u << 0)
#define FLAG_PUMP_P1    (1u << 1)
#define FLAG_PUMP_P2    (1u << 2)
#define FLAG_PUMP_P3    (1u << 3)
#define FLAG_PUMP_POND  (1u << 4)

// Command payload — carried in MSG_COMMAND packets.
struct __attribute__((packed)) CommandData {
    uint8_t target_pump;  // Which pump: 1=P1, 2=P2, 3=P3, 4=Pond pump
    uint8_t action;       // 0 = turn OFF,  1 = turn ON
    uint8_t reserved;     // Unused — keep zero
};  // 3 bytes

// Configuration parameters — stored in NVS (flash), exchanged via CONFIG_GET/SET/RESP.
struct __attribute__((packed)) NodeConfig {
    uint32_t pump_min_runtime_ms;        // Pump must run at least this long before stopping (ms)
    uint32_t pump_min_cooldown_ms;       // Pump must be off at least this long before starting (ms)
    uint32_t replenish_runon_ms;         // How long the pond pump runs during a tank refill (ms)
    uint32_t telemetry_interval_ms;      // How often to send telemetry (ms)  default = 10 000
    uint32_t network_timeout_ms;         // After this long without hearing a peer, raise error (ms)
    uint32_t cmd_response_timeout_ms;    // How long to wait for pond telemetry after a command (ms)
    uint8_t  boot_auto_mode;             // 1 = start in auto mode after reboot,  0 = start in manual
    uint8_t  _reserved[7];              // Future use — always zero
};  // 32 bytes total

// Operational statistics — persisted in NVS, readable via STATS_GET/RESP.
struct __attribute__((packed)) StatsPayload {
    uint32_t uptime_s;         // Seconds since last reboot (not saved to NVS — computed at read time)
    uint32_t runtime_p1_s;     // Total seconds pump P1 has been running (accumulated)
    uint32_t runtime_p2_s;
    uint32_t runtime_p3_s;
    uint32_t runtime_pond_s;   // Total seconds pond pump has been running
    uint16_t fill_cycles;      // How many tank refills have happened (rising count = possible leak)
    uint16_t boot_count;       // Total reboots
    uint8_t  last_fault;       // Error code at the most recent fault event
    uint8_t  reserved[7];
};  // 32 bytes total

// The full packet: header + largest possible payload.
struct __attribute__((packed)) LoRaPacket {
    LoRaHeader header;    // 6 bytes
    union {
        TelemetryData telemetry;   //  8 bytes
        CommandData   command;     //  3 bytes
        NodeConfig    config;      // 32 bytes  ← largest member, sets union size
        StatsPayload  stats;       // 32 bytes
    } payload;            // 32 bytes
};  // 38 bytes total

static_assert(sizeof(LoRaPacket)   <= 255, "LoRaPacket exceeds maximum RadioLib payload");
static_assert(sizeof(NodeConfig)   == 32,  "NodeConfig size changed — bump CONFIG_VERSION");
static_assert(sizeof(StatsPayload) == 32,  "StatsPayload size mismatch");

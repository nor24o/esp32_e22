#pragma once

#include <stdint.h>
#include <string.h>

// =============================================================================
// PACKED NETWORK PROTOCOL STRUCTURES
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

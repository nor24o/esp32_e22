#pragma once

#include "globals.hpp"
#include "nvs_manager.hpp"

// =============================================================================
// PACKET BUILDER HELPERS
//
// These functions build a LoRa packet and drop it into the TX queue.
// The LoRaTransceiver task picks packets from the queue and sends them.
// All helpers fill in the header fields so callers just set the payload.
// =============================================================================

// Sends the current system state to the gateway as a telemetry packet.
// Called every telemetry_interval_ms by the ControlEngine.
static void buildAndQueueTelemetry() {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.header.magic_word = MY_NETWORK_MAGIC;
    pkt.header.target_id  = NODE_GATEWAY;        // telemetry goes to the gateway
    pkt.header.sender_id  = NODE_TANK_LOCAL;
    pkt.header.msg_id     = nextMsgId();
    pkt.header.msg_type   = MSG_TELEMETRY;

    // Take a snapshot of the shared state under the mutex.
    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        uint8_t flags = 0;
        if (gState.autoMode)  flags |= FLAG_AUTO_MODE;
        if (gState.relay_p1)  flags |= FLAG_PUMP_P1;
        if (gState.relay_p2)  flags |= FLAG_PUMP_P2;
        if (gState.relay_p3)  flags |= FLAG_PUMP_P3;
        if (gState.pondPump)  flags |= FLAG_PUMP_POND;

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

// Sends a pump ON or OFF command to another node (e.g. tank → pond).
// Returns the msg_id used so the caller can track the pending command.
static uint8_t sendPumpCommand(uint8_t target_node, uint8_t pump_id, uint8_t action) {
    LoRaPacket pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.header.magic_word           = MY_NETWORK_MAGIC;
    pkt.header.target_id            = target_node;
    pkt.header.sender_id            = NODE_TANK_LOCAL;
    pkt.header.msg_id               = nextMsgId();
    pkt.header.msg_type             = MSG_COMMAND;
    pkt.payload.command.target_pump = pump_id;
    pkt.payload.command.action      = action;
    pkt.payload.command.reserved    = 0;

    xQueueSend(xTxQueue, &pkt, 0);
    return pkt.header.msg_id;
}

// Sends the current gConfig to the requesting node.
// Called in response to MSG_CONFIG_GET or MSG_CONFIG_SET.
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

// Sends operational statistics to the requesting node.
// Called in response to MSG_STATS_GET.
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

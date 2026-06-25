#pragma once

#include "globals.hpp"
#include "nvs_manager.hpp"

// =============================================================================
// SHARED HELPER FUNCTIONS
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

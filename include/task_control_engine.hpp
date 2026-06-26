#pragma once

#include "globals.hpp"
#include "nvs_manager.hpp"
#include "helpers.hpp"
#include "pump_hysteresis.hpp"

// =============================================================================
// TASK: CONTROL ENGINE  (Priority 4, Core 1, 50 ms)
// =============================================================================

static void Task_ControlEngine(void *pvParams) {
    PumpHysteresis ph_p1   = { false, 0, 0, false };
    PumpHysteresis ph_p2   = { false, 0, 0, false };
    PumpHysteresis ph_p3   = { false, 0, 0, false };
    PumpHysteresis ph_pond = { false, 0, 0, false };

    uint32_t onSince_p1   = 0;
    uint32_t onSince_p2   = 0;
    uint32_t onSince_p3   = 0;
    uint32_t onSince_pond = 0;

    bool     replenishActive    = false;
    uint32_t replenishStartTime = 0;

    bool     awaitingAckInternal = false;
    uint32_t ackSentTime_ms      = 0;
    uint8_t  ackRetryCount       = 0;
    uint8_t  pendingPondAction   = 0;

    uint32_t lastTelemetryTime_ms = 0;
    uint32_t lastStatusPrint_ms   = 0;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));

        uint32_t now = millis();

        // ── Snapshot global state ───────────────────────────────────────────
        uint8_t  waterLevel, errorCode;
        bool     autoMode;
        uint32_t lastGateway, lastPond;
        bool     btnMode, btnP1, btnP2, btnP3, btnPond;

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) != pdTRUE) continue;

        waterLevel   = gState.waterLevel;
        autoMode     = gState.autoMode;
        errorCode    = gState.errorCode;
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
                errorCode == 3 ? "NO-COMMS" : "OK";
            Serial.printf("\r[%6lus] WL:%u %s | P1:%s P2:%s P3:%s Pond:%s | %s     ",
                now / 1000,
                waterLevel,
                autoMode ? "AUTO" : "MAN ",
                ph_p1.state   ? "ON " : "off",
                ph_p2.state   ? "ON " : "off",
                ph_p3.state   ? "ON " : "off",
                ph_pond.state ? "ON " : "off",
                errStr);
        }

        // ── Network-timeout assessment ───────────────────────────────────────
        bool networkOk = !gRadioOk ||
            (((now - lastGateway) < gConfig.network_timeout_ms) &&
             ((now - lastPond)    < gConfig.network_timeout_ms));

        // ── Mode toggle ──────────────────────────────────────────────────────
        if (btnMode) {
            autoMode = !autoMode;
            if (!autoMode && replenishActive) replenishActive = false;
            gConfig.boot_auto_mode = autoMode ? 1 : 0;
            saveConfig();
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

                if (rxPkt.header.sender_id == NODE_GATEWAY) {
                    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        gState.lastGatewayContact_ms = now;
                        xSemaphoreGive(xStateMutex);
                    }
                }

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
                    case 4:
                        // Pond pump relay: tank forwards to NODE_POND_REMOTE
                        if (gRadioOk) {
                            uint8_t act = cmd.action;
                            if (act == 1 && canTurnPumpOn(ph_pond)) {
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
                            } else if (act == 0 && canTurnPumpOff(ph_pond)) {
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
                    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        gState.autoMode = (gConfig.boot_auto_mode != 0);
                        autoMode        = gState.autoMode;
                        xSemaphoreGive(xStateMutex);
                    }
                }
                // Always respond so the gateway can detect if validation rejected the set.
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
                    saveStats();
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

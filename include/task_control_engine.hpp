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

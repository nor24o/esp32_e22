#pragma once

#include "globals.hpp"
#include "nvs_manager.hpp"
#include "helpers.hpp"
#include "pump_hysteresis.hpp"

// =============================================================================
// TASK: CONTROL ENGINE  (Priority 4, Core 1, runs every 50 ms)
//
// This is the "brain" of the tank controller.  It:
//   1. Reads received LoRa packets from xRxQueue and acts on commands.
//   2. In AUTO mode: turns the pond pump on when water is low, off when full.
//   3. In MANUAL mode: responds to touch button presses.
//   4. Sends telemetry to the gateway every telemetry_interval_ms.
//   5. Tracks whether the pond confirmed the last command via its telemetry.
//
// GATEWAY COMMANDS ARE ALWAYS OBEYED — regardless of auto/manual mode.
// The gateway is the master of the system.
//
// POND COMMAND CONFIRMATION (no ACK packets needed)
// ─────────────────────────────────────────────────
// When the tank sends a pond pump command, it sets a "pending" flag and starts
// a timer (cmd_response_timeout_ms).  The pond sends telemetry every 10 s.
// When that telemetry arrives and shows the expected pump state → confirmed.
// If the timer expires before a matching telemetry arrives → warning logged,
// no retry.  The next auto-mode cycle or manual press will issue a new command
// if the situation still calls for it.
// =============================================================================

static void Task_ControlEngine(void *pvParams) {

    // ── Per-pump hysteresis trackers ─────────────────────────────────────────
    // Each pump enforces minimum ON/OFF times independently.
    PumpHysteresis ph_p1   = { false, 0, 0, false };
    PumpHysteresis ph_p2   = { false, 0, 0, false };
    PumpHysteresis ph_p3   = { false, 0, 0, false };
    PumpHysteresis ph_pond = { false, 0, 0, false };

    // ── Pump runtime tracking ─────────────────────────────────────────────────
    // onSince stores millis() when the pump started; 0 = not tracked.
    uint32_t onSince_p1   = 0;
    uint32_t onSince_p2   = 0;
    uint32_t onSince_p3   = 0;
    uint32_t onSince_pond = 0;

    // ── Tank-refill (replenish) state ─────────────────────────────────────────
    // In AUTO mode, when the tank water level drops to 0, the control engine
    // starts the pond pump to refill the tank.  It runs for replenish_runon_ms,
    // then stops once the tank is full (all three float switches active).
    bool     replenishActive    = false;
    uint32_t replenishStartTime = 0;

    // ── Pond command response window ──────────────────────────────────────────
    // After the tank sends a pump command to the pond, we set pondCmdPending.
    // We then watch incoming pond telemetry.  If the telemetry shows the right
    // pump state before pondCmdDeadline_ms, the command is confirmed.
    // If the deadline passes, we log a warning and clear the pending flag.
    bool     pondCmdPending       = false;
    bool     pondCmdExpectedState = false;   // the state we commanded (true=ON)
    uint32_t pondCmdDeadline_ms   = 0;       // absolute timestamp of the deadline

    // ── Telemetry and console timers ─────────────────────────────────────────
    uint32_t lastTelemetryTime_ms = 0;
    uint32_t lastStatusPrint_ms   = 0;

    for (;;) {
        // Wake on notification from LoRaTransceiver (new RX packet), or after 50 ms.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));

        uint32_t now = millis();

        // ── Snapshot shared state ─────────────────────────────────────────────
        // Take the mutex briefly to copy what we need, then release it.
        // This keeps the mutex held for as little time as possible.
        uint8_t  waterLevel, errorCode;
        bool     autoMode;
        uint32_t lastGateway, lastPond;
        bool     btnMode, btnP1, btnP2, btnP3, btnPond;

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) != pdTRUE) continue;

        waterLevel  = gState.waterLevel;
        autoMode    = gState.autoMode;
        errorCode   = gState.errorCode;
        lastGateway = gState.lastGatewayContact_ms;
        lastPond    = gState.lastPondContact_ms;

        // Read and clear button edges — "edge" means the button was just pressed.
        btnMode = gState.btnMode_edge; gState.btnMode_edge = false;
        btnP1   = gState.btnP1_edge;   gState.btnP1_edge   = false;
        btnP2   = gState.btnP2_edge;   gState.btnP2_edge   = false;
        btnP3   = gState.btnP3_edge;   gState.btnP3_edge   = false;
        btnPond = gState.btnPond_edge;  gState.btnPond_edge  = false;

        xSemaphoreGive(xStateMutex);

        // ── Console status line (printed every 2 s) ───────────────────────────
        if ((now - lastStatusPrint_ms) >= 2000) {
            lastStatusPrint_ms = now;
            Serial.printf("\r[%6lus] WL:%u %s | P1:%s P2:%s P3:%s Pond:%s | %s%s     ",
                now / 1000,
                waterLevel,
                autoMode ? "AUTO" : "MAN ",
                ph_p1.state   ? "ON " : "off",
                ph_p2.state   ? "ON " : "off",
                ph_p3.state   ? "ON " : "off",
                ph_pond.state ? "ON " : "off",
                errorCode == 3 ? "NO-COMMS" : "OK",
                pondCmdPending ? " [POND-CMD-PENDING]" : "");
        }

        // ── Network timeout check ─────────────────────────────────────────────
        // If LoRa is working and we have not heard from either peer within
        // network_timeout_ms, set errorCode to 3 (no-comms).
        bool networkOk = !gRadioOk ||
            (((now - lastGateway) < gConfig.network_timeout_ms) &&
             ((now - lastPond)    < gConfig.network_timeout_ms));

        // ── MODE button ───────────────────────────────────────────────────────
        if (btnMode) {
            autoMode = !autoMode;
            if (!autoMode && replenishActive) {
                replenishActive = false;   // cancel any active refill when switching to manual
            }
            gConfig.boot_auto_mode = autoMode ? 1 : 0;
            saveConfig();
            if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                gState.autoMode = autoMode;
                xSemaphoreGive(xStateMutex);
            }
            Serial.printf("\n[MODE] Switched to %s mode\n", autoMode ? "AUTO" : "MANUAL");
        }

        // ====================================================================
        // PROCESS RECEIVED PACKETS
        // ====================================================================
        LoRaPacket rxPkt;
        while (xQueueReceive(xRxQueue, &rxPkt, 0) == pdTRUE) {

            // ── Pump command received ─────────────────────────────────────────
            // Commands come from the gateway (P1/P2/P3/Pond) or from the gateway
            // relaying a pond pump request.  GATEWAY COMMANDS ARE ALWAYS OBEYED.
            if (rxPkt.header.msg_type == MSG_COMMAND) {

                if (!isValidMsgId(rxPkt.header.sender_id, rxPkt.header.msg_id)) {
                    // Duplicate packet — ignore
                    continue;
                }

                // Update the gateway contact timestamp any time we hear from it.
                if (rxPkt.header.sender_id == NODE_GATEWAY) {
                    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        gState.lastGatewayContact_ms = now;
                        xSemaphoreGive(xStateMutex);
                    }
                }

                CommandData &cmd = rxPkt.payload.command;

                switch (cmd.target_pump) {

                    case 1:  // Pump P1 (local relay)
                        if (cmd.action == 1 && canTurnPumpOn(ph_p1)) {
                            setPumpState(ph_p1, true, RELAY_P1);
                            trackPumpOn(onSince_p1);
                            Serial.printf("\n[CMD] P1 → ON\n");
                        } else if (cmd.action == 0 && canTurnPumpOff(ph_p1)) {
                            gStats.runtime_p1_s += trackPumpOff(onSince_p1);
                            setPumpState(ph_p1, false, RELAY_P1);
                            saveStats();
                            Serial.printf("\n[CMD] P1 → OFF\n");
                        }
                        break;

                    case 2:  // Pump P2 (local relay)
                        if (cmd.action == 1 && canTurnPumpOn(ph_p2)) {
                            setPumpState(ph_p2, true, RELAY_P2);
                            trackPumpOn(onSince_p2);
                            Serial.printf("\n[CMD] P2 → ON\n");
                        } else if (cmd.action == 0 && canTurnPumpOff(ph_p2)) {
                            gStats.runtime_p2_s += trackPumpOff(onSince_p2);
                            setPumpState(ph_p2, false, RELAY_P2);
                            saveStats();
                            Serial.printf("\n[CMD] P2 → OFF\n");
                        }
                        break;

                    case 3:  // Pump P3 (local relay)
                        if (cmd.action == 1 && canTurnPumpOn(ph_p3)) {
                            setPumpState(ph_p3, true, RELAY_P3);
                            trackPumpOn(onSince_p3);
                            Serial.printf("\n[CMD] P3 → ON\n");
                        } else if (cmd.action == 0 && canTurnPumpOff(ph_p3)) {
                            gStats.runtime_p3_s += trackPumpOff(onSince_p3);
                            setPumpState(ph_p3, false, RELAY_P3);
                            saveStats();
                            Serial.printf("\n[CMD] P3 → OFF\n");
                        }
                        break;

                    case 4: {
                        // Pond pump — forward the command to the pond controller via LoRa.
                        // The pond will confirm by including its pump state in its telemetry.
                        if (!gRadioOk) break;

                        bool wantOn = (cmd.action == 1);

                        if (wantOn && canTurnPumpOn(ph_pond)) {
                            // If a refill was active, the gateway is taking over — cancel it.
                            if (replenishActive) replenishActive = false;

                            setPumpState(ph_pond, true, -1);
                            trackPumpOn(onSince_pond);

                            // Send command to pond and open the response window.
                            sendPumpCommand(NODE_POND_REMOTE, 1, 1);
                            pondCmdPending       = true;
                            pondCmdExpectedState = true;
                            pondCmdDeadline_ms   = now + gConfig.cmd_response_timeout_ms;
                            Serial.printf("\n[POND] Commanded ON — waiting up to %lu s for telemetry confirmation\n",
                                          gConfig.cmd_response_timeout_ms / 1000);

                            if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                                gState.pondPump        = true;
                                gState.pondCmdPending  = true;
                                xSemaphoreGive(xStateMutex);
                            }

                        } else if (!wantOn && canTurnPumpOff(ph_pond)) {
                            if (replenishActive) replenishActive = false;

                            gStats.runtime_pond_s += trackPumpOff(onSince_pond);
                            setPumpState(ph_pond, false, -1);
                            saveStats();

                            sendPumpCommand(NODE_POND_REMOTE, 1, 0);
                            pondCmdPending       = true;
                            pondCmdExpectedState = false;
                            pondCmdDeadline_ms   = now + gConfig.cmd_response_timeout_ms;
                            Serial.printf("\n[POND] Commanded OFF — waiting up to %lu s for telemetry confirmation\n",
                                          gConfig.cmd_response_timeout_ms / 1000);

                            if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                                gState.pondPump        = false;
                                gState.pondCmdPending  = true;
                                xSemaphoreGive(xStateMutex);
                            }
                        }
                        break;
                    }

                    default:
                        Serial.printf("\n[CMD] Unknown pump id %u — ignored\n", cmd.target_pump);
                        break;
                }

                // Keep relay states synced in shared state after any command.
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    gState.relay_p1 = ph_p1.state;
                    gState.relay_p2 = ph_p2.state;
                    gState.relay_p3 = ph_p3.state;
                    xSemaphoreGive(xStateMutex);
                }

            // ── Telemetry received from the pond ──────────────────────────────
            // The pond broadcasts its state every 10 s.  We use this to:
            //   1. Know the pond is still alive (update lastPondContact_ms).
            //   2. Confirm that a pump command we sent earlier was applied.
            } else if (rxPkt.header.msg_type == MSG_TELEMETRY &&
                       rxPkt.header.sender_id == NODE_POND_REMOTE) {

                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    gState.lastPondContact_ms = now;
                    xSemaphoreGive(xStateMutex);
                }

                // Does the pond's reported pump state match what we commanded?
                if (pondCmdPending) {
                    bool pondPumpActual = (rxPkt.payload.telemetry.system_flags & FLAG_PUMP_POND) != 0;

                    if (pondPumpActual == pondCmdExpectedState) {
                        pondCmdPending = false;
                        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                            gState.pondCmdPending = false;
                            xSemaphoreGive(xStateMutex);
                        }
                        Serial.printf("\n[POND] Command confirmed — pond pump is %s\n",
                                      pondCmdExpectedState ? "ON" : "OFF");
                    }
                    // If it does not match yet, the pond might still be in the middle of
                    // switching.  Just wait — the deadline will catch a true failure.
                }

            // ── Telemetry from any other node ─────────────────────────────────
            } else if (rxPkt.header.msg_type == MSG_TELEMETRY &&
                       rxPkt.header.sender_id == NODE_GATEWAY) {

                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    gState.lastGatewayContact_ms = now;
                    xSemaphoreGive(xStateMutex);
                }

            // ── Config requests from the gateway ──────────────────────────────
            } else if (rxPkt.header.msg_type == MSG_CONFIG_GET) {

                if (!isValidMsgId(rxPkt.header.sender_id, rxPkt.header.msg_id)) continue;
                sendConfigResp(rxPkt.header.sender_id);
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    gState.lastGatewayContact_ms = now;
                    xSemaphoreGive(xStateMutex);
                }

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
                    Serial.println("\n[CFG] New config applied and saved");
                } else {
                    Serial.println("\n[CFG] Received config REJECTED (values out of range)");
                }
                // Always reply so the gateway can see what the tank actually accepted.
                sendConfigResp(rxPkt.header.sender_id);
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    gState.lastGatewayContact_ms = now;
                    xSemaphoreGive(xStateMutex);
                }

            } else if (rxPkt.header.msg_type == MSG_STATS_GET) {

                if (!isValidMsgId(rxPkt.header.sender_id, rxPkt.header.msg_id)) continue;
                sendStatsResp(rxPkt.header.sender_id);
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    gState.lastGatewayContact_ms = now;
                    xSemaphoreGive(xStateMutex);
                }
            }
        }

        // ── Pond command response window expiry ───────────────────────────────
        if (pondCmdPending && (now >= pondCmdDeadline_ms)) {
            pondCmdPending = false;
            if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                gState.pondCmdPending = false;
                xSemaphoreGive(xStateMutex);
            }
            Serial.printf("\n[POND] WARNING: no telemetry confirmation within %lu s — "
                          "pond may not have received the command\n",
                          gConfig.cmd_response_timeout_ms / 1000);
        }

        // ── Network-timeout error code ────────────────────────────────────────
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (!networkOk && gState.errorCode == 0) gState.errorCode = 3;
            if ( networkOk && gState.errorCode == 3) gState.errorCode = 0;
            xSemaphoreGive(xStateMutex);
        }

        // ====================================================================
        // AUTO MODE — automatic tank refill
        // ====================================================================
        if (autoMode) {

            if (waterLevel == 0 && !replenishActive) {
                // Tank is empty — start filling if the pond pump cooldown allows it.
                if (canTurnPumpOn(ph_pond)) {
                    replenishActive    = true;
                    replenishStartTime = now;
                    gStats.fill_cycles++;
                    saveStats();
                    setPumpState(ph_pond, true, -1);
                    trackPumpOn(onSince_pond);
                    Serial.printf("\n[AUTO] Tank empty — starting refill (fill cycle #%u)\n",
                                  gStats.fill_cycles);

                    if (gRadioOk) {
                        sendPumpCommand(NODE_POND_REMOTE, 1, 1);
                        pondCmdPending       = true;
                        pondCmdExpectedState = true;
                        pondCmdDeadline_ms   = now + gConfig.cmd_response_timeout_ms;
                        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                            gState.pondPump       = true;
                            gState.pondCmdPending = true;
                            xSemaphoreGive(xStateMutex);
                        }
                    }
                }

            } else if (replenishActive) {
                // Filling is in progress — stop when the tank is full AND the minimum
                // run time has elapsed.
                bool runOnExpired = (now - replenishStartTime) >= gConfig.replenish_runon_ms;
                if (runOnExpired && waterLevel >= 3 && canTurnPumpOff(ph_pond)) {
                    replenishActive = false;
                    gStats.runtime_pond_s += trackPumpOff(onSince_pond);
                    setPumpState(ph_pond, false, -1);
                    saveStats();
                    Serial.println("\n[AUTO] Tank full — refill complete, stopping pond pump");

                    if (gRadioOk) {
                        sendPumpCommand(NODE_POND_REMOTE, 1, 0);
                        pondCmdPending       = true;
                        pondCmdExpectedState = false;
                        pondCmdDeadline_ms   = now + gConfig.cmd_response_timeout_ms;
                        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                            gState.pondPump       = false;
                            gState.pondCmdPending = true;
                            xSemaphoreGive(xStateMutex);
                        }
                    }
                }
            }
        }

        // ====================================================================
        // MANUAL MODE — touch button presses
        // ====================================================================
        if (!autoMode) {

            // Helper lambda to toggle a local relay pump.
            // We can't use a real lambda here easily with the params, so inline it:
            if (btnP1) {
                bool wantOn = !ph_p1.state;
                if (wantOn && canTurnPumpOn(ph_p1)) {
                    setPumpState(ph_p1, true, RELAY_P1);
                    trackPumpOn(onSince_p1);
                    Serial.println("\n[BTN] P1 → ON");
                } else if (!wantOn && canTurnPumpOff(ph_p1)) {
                    gStats.runtime_p1_s += trackPumpOff(onSince_p1);
                    setPumpState(ph_p1, false, RELAY_P1);
                    saveStats();
                    Serial.println("\n[BTN] P1 → OFF");
                }
            }

            if (btnP2) {
                bool wantOn = !ph_p2.state;
                if (wantOn && canTurnPumpOn(ph_p2)) {
                    setPumpState(ph_p2, true, RELAY_P2);
                    trackPumpOn(onSince_p2);
                    Serial.println("\n[BTN] P2 → ON");
                } else if (!wantOn && canTurnPumpOff(ph_p2)) {
                    gStats.runtime_p2_s += trackPumpOff(onSince_p2);
                    setPumpState(ph_p2, false, RELAY_P2);
                    saveStats();
                    Serial.println("\n[BTN] P2 → OFF");
                }
            }

            if (btnP3) {
                bool wantOn = !ph_p3.state;
                if (wantOn && canTurnPumpOn(ph_p3)) {
                    setPumpState(ph_p3, true, RELAY_P3);
                    trackPumpOn(onSince_p3);
                    Serial.println("\n[BTN] P3 → ON");
                } else if (!wantOn && canTurnPumpOff(ph_p3)) {
                    gStats.runtime_p3_s += trackPumpOff(onSince_p3);
                    setPumpState(ph_p3, false, RELAY_P3);
                    saveStats();
                    Serial.println("\n[BTN] P3 → OFF");
                }
            }

            if (btnPond && gRadioOk) {
                bool wantOn = !ph_pond.state;
                if (wantOn && canTurnPumpOn(ph_pond)) {
                    setPumpState(ph_pond, true, -1);
                    trackPumpOn(onSince_pond);
                    sendPumpCommand(NODE_POND_REMOTE, 1, 1);
                    pondCmdPending       = true;
                    pondCmdExpectedState = true;
                    pondCmdDeadline_ms   = now + gConfig.cmd_response_timeout_ms;
                    Serial.println("\n[BTN] Pond pump → ON (command sent, waiting for confirmation)");
                    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        gState.pondPump       = true;
                        gState.pondCmdPending = true;
                        xSemaphoreGive(xStateMutex);
                    }
                } else if (!wantOn && canTurnPumpOff(ph_pond)) {
                    gStats.runtime_pond_s += trackPumpOff(onSince_pond);
                    setPumpState(ph_pond, false, -1);
                    saveStats();
                    sendPumpCommand(NODE_POND_REMOTE, 1, 0);
                    pondCmdPending       = true;
                    pondCmdExpectedState = false;
                    pondCmdDeadline_ms   = now + gConfig.cmd_response_timeout_ms;
                    Serial.println("\n[BTN] Pond pump → OFF (command sent, waiting for confirmation)");
                    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        gState.pondPump       = false;
                        gState.pondCmdPending = true;
                        xSemaphoreGive(xStateMutex);
                    }
                }
            }
        }

        // ── Push latest relay states back into shared state ───────────────────
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            gState.relay_p1 = ph_p1.state;
            gState.relay_p2 = ph_p2.state;
            gState.relay_p3 = ph_p3.state;
            gState.pondPump = ph_pond.state;
            xSemaphoreGive(xStateMutex);
        }

        // ── Periodic telemetry to gateway ─────────────────────────────────────
        if ((now - lastTelemetryTime_ms) >= gConfig.telemetry_interval_ms) {
            lastTelemetryTime_ms = now;
            buildAndQueueTelemetry();
        }
    }
}

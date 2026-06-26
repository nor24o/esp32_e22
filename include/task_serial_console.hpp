#pragma once

#include <esp_system.h>
#include "globals.hpp"
#include "nvs_manager.hpp"
#include "helpers.hpp"

// =============================================================================
// TASK: SERIAL CONSOLE  (Priority 1, Core 0)
//
// Reads commands from the USB serial port (115200 baud).
// Type a command and press Enter.
//
// This is mainly useful during development and debugging.
// =============================================================================

static void handleConsoleCommand(const char *cmd) {
    if (strlen(cmd) == 0) return;
    Serial.print('\n');

    if (strcmp(cmd, "help") == 0) {
        Serial.println("[CON] Available commands:");
        Serial.println("[CON]   status  — show current system state");
        Serial.println("[CON]   radio   — show LoRa signal quality");
        Serial.println("[CON]   config  — show all config values");
        Serial.println("[CON]   stats   — show pump runtime counters");
        Serial.println("[CON]   tx      — send a telemetry packet now");
        Serial.println("[CON]   auto    — switch to AUTO mode (saved to flash)");
        Serial.println("[CON]   manual  — switch to MANUAL mode (saved to flash)");
        Serial.println("[CON]   p1      — toggle pump P1 (manual mode only)");
        Serial.println("[CON]   p2      — toggle pump P2 (manual mode only)");
        Serial.println("[CON]   p3      — toggle pump P3 (manual mode only)");
        Serial.println("[CON]   pond    — toggle pond pump (manual mode only)");
        Serial.println("[CON]   reboot  — restart the device");

    } else if (strcmp(cmd, "status") == 0) {
        SystemState s;
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            s = gState;
            xSemaphoreGive(xStateMutex);
        }
        uint32_t now = millis();
        Serial.printf("[CON] Status at %lu s uptime\n", now / 1000);
        Serial.printf("[CON]   Mode        : %s\n",      s.autoMode ? "AUTO" : "MANUAL");
        Serial.printf("[CON]   Water level : %u / 3\n",  s.waterLevel);
        Serial.printf("[CON]   Pumps P1/P2/P3 : %s / %s / %s\n",
                      s.relay_p1 ? "ON" : "off",
                      s.relay_p2 ? "ON" : "off",
                      s.relay_p3 ? "ON" : "off");
        Serial.printf("[CON]   Pond pump   : %s%s\n",
                      s.pondPump ? "ON" : "off",
                      s.pondCmdPending ? " (waiting for pond confirmation)" : "");
        Serial.printf("[CON]   Error code  : %s\n",      s.errorCode == 3 ? "3 = no-comms" : "0 = OK");
        Serial.printf("[CON]   Gateway last seen : %lu s ago\n", (now - s.lastGatewayContact_ms) / 1000);
        Serial.printf("[CON]   Pond last seen    : %lu s ago\n", (now - s.lastPondContact_ms) / 1000);
        Serial.printf("[CON]   Signal      : RSSI=%d dBm  SNR=%d dB  Radio=%s\n",
                      s.lastRssi, s.lastSnr, gRadioOk ? "OK" : "FAILED");

    } else if (strcmp(cmd, "radio") == 0) {
        int8_t rssi, snr;
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            rssi = gState.lastRssi;
            snr  = gState.lastSnr;
            xSemaphoreGive(xStateMutex);
        }
        Serial.printf("[CON] Radio: %s | RSSI=%d dBm | SNR=%d dB\n",
                      gRadioOk ? "OK" : "FAILED", rssi, snr);

    } else if (strcmp(cmd, "tx") == 0) {
        if (!gRadioOk) {
            Serial.println("[CON] Radio not available — cannot send");
        } else {
            buildAndQueueTelemetry();
            Serial.println("[CON] Telemetry queued");
        }

    } else if (strcmp(cmd, "auto") == 0) {
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            gState.autoMode = true;
            xSemaphoreGive(xStateMutex);
        }
        gConfig.boot_auto_mode = 1;
        saveConfig();
        Serial.println("[CON] Switched to AUTO mode (saved)");

    } else if (strcmp(cmd, "manual") == 0) {
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            gState.autoMode = false;
            xSemaphoreGive(xStateMutex);
        }
        gConfig.boot_auto_mode = 0;
        saveConfig();
        Serial.println("[CON] Switched to MANUAL mode (saved)");

    } else if (strcmp(cmd, "p1") == 0 || strcmp(cmd, "p2") == 0 ||
               strcmp(cmd, "p3") == 0 || strcmp(cmd, "pond") == 0) {
        bool injected = false;
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (!gState.autoMode) {
                if      (strcmp(cmd, "p1")   == 0) gState.btnP1_edge   = true;
                else if (strcmp(cmd, "p2")   == 0) gState.btnP2_edge   = true;
                else if (strcmp(cmd, "p3")   == 0) gState.btnP3_edge   = true;
                else if (strcmp(cmd, "pond") == 0) gState.btnPond_edge = true;
                injected = true;
            }
            xSemaphoreGive(xStateMutex);
        }
        if (injected) {
            Serial.printf("[CON] Toggle %s — will apply on next control cycle\n", cmd);
        } else {
            Serial.println("[CON] Must be in MANUAL mode first — type 'manual'");
        }

    } else if (strcmp(cmd, "config") == 0) {
        Serial.printf("[CON] Config (version %u):\n", CONFIG_VERSION);
        Serial.printf("[CON]   pump_min_runtime_ms       = %lu ms (%lu s)\n",
                      gConfig.pump_min_runtime_ms, gConfig.pump_min_runtime_ms / 1000);
        Serial.printf("[CON]   pump_min_cooldown_ms      = %lu ms (%lu s)\n",
                      gConfig.pump_min_cooldown_ms, gConfig.pump_min_cooldown_ms / 1000);
        Serial.printf("[CON]   replenish_runon_ms        = %lu ms (%lu s)\n",
                      gConfig.replenish_runon_ms, gConfig.replenish_runon_ms / 1000);
        Serial.printf("[CON]   telemetry_interval_ms     = %lu ms (%lu s)\n",
                      gConfig.telemetry_interval_ms, gConfig.telemetry_interval_ms / 1000);
        Serial.printf("[CON]   network_timeout_ms        = %lu ms (%lu s)\n",
                      gConfig.network_timeout_ms, gConfig.network_timeout_ms / 1000);
        Serial.printf("[CON]   cmd_response_timeout_ms   = %lu ms (%lu s)\n",
                      gConfig.cmd_response_timeout_ms, gConfig.cmd_response_timeout_ms / 1000);
        Serial.printf("[CON]   boot_auto_mode            = %u\n",
                      gConfig.boot_auto_mode);

    } else if (strcmp(cmd, "stats") == 0) {
        Serial.printf("[CON] Stats (boot #%u, uptime %lu s):\n",
                      gStats.boot_count, millis() / 1000);
        Serial.printf("[CON]   P1 runtime  : %lu s\n",  gStats.runtime_p1_s);
        Serial.printf("[CON]   P2 runtime  : %lu s\n",  gStats.runtime_p2_s);
        Serial.printf("[CON]   P3 runtime  : %lu s\n",  gStats.runtime_p3_s);
        Serial.printf("[CON]   Pond runtime: %lu s\n",  gStats.runtime_pond_s);
        Serial.printf("[CON]   Fill cycles : %u\n",     gStats.fill_cycles);
        Serial.printf("[CON]   Last fault  : %u\n",     gStats.last_fault);

    } else if (strcmp(cmd, "reboot") == 0) {
        Serial.println("[CON] Rebooting...");
        Serial.flush();
        esp_restart();

    } else {
        Serial.printf("[CON] Unknown command '%s' — type 'help' for a list\n", cmd);
    }
}

static void Task_SerialConsole(void *pvParams) {
    char    buf[64];
    uint8_t idx = 0;

    for (;;) {
        while (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\r') continue;
            if (c == '\n' || idx >= (uint8_t)(sizeof(buf) - 1)) {
                buf[idx] = '\0';
                idx = 0;
                const char *p = buf;
                while (*p == ' ') p++;   // strip leading spaces
                handleConsoleCommand(p);
            } else {
                buf[idx++] = c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

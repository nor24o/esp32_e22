#pragma once

#include <esp_system.h>
#include "globals.hpp"
#include "nvs_manager.hpp"
#include "helpers.hpp"

// =============================================================================
// TASK: SERIAL CONSOLE  (Priority 1, Core 0)
// =============================================================================

static void handleConsoleCommand(const char *cmd) {
    if (strlen(cmd) == 0) return;
    Serial.print('\n');

    if (strcmp(cmd, "help") == 0) {
        Serial.println("[CON] Commands:");
        Serial.println("[CON]   status        — system state snapshot");
        Serial.println("[CON]   radio         — LoRa signal quality");
        Serial.println("[CON]   config        — dump runtime config (gConfig)");
        Serial.println("[CON]   stats         — dump pump stats (gStats)");
        Serial.println("[CON]   tx            — queue immediate telemetry broadcast");
        Serial.println("[CON]   auto          — enter AUTO mode");
        Serial.println("[CON]   manual        — enter MANUAL mode");
        Serial.println("[CON]   p1/p2/p3/pond — toggle pump relay (manual mode only)");
        Serial.println("[CON]   reboot        — software reset");

    } else if (strcmp(cmd, "status") == 0) {
        SystemState s;
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            s = gState;
            xSemaphoreGive(xStateMutex);
        }
        uint32_t now = millis();
        Serial.printf("[CON] Status @ %lus\n", now / 1000);
        Serial.printf("[CON]   Mode    : %s  |  WaterLevel: %u\n",
                      s.autoMode ? "AUTO" : "MANUAL", s.waterLevel);
        Serial.printf("[CON]   Relays  : P1=%s  P2=%s  P3=%s  Pond=%s\n",
                      s.relay_p1 ? "ON" : "off", s.relay_p2 ? "ON" : "off",
                      s.relay_p3 ? "ON" : "off", s.pondPump  ? "ON" : "off");
        Serial.printf("[CON]   Error   : %u  |  AwaitAck: %s\n",
                      s.errorCode, s.awaitingAck ? "YES" : "no");
        Serial.printf("[CON]   GW seen : %lus ago  |  Pond: %lus ago\n",
                      (now - s.lastGatewayContact_ms) / 1000,
                      (now - s.lastPondContact_ms)    / 1000);
        Serial.printf("[CON]   Signal  : RSSI=%d dBm  SNR=%d dB  Radio=%s\n",
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
            Serial.println("[CON] Radio not available — packet not queued");
        } else {
            buildAndQueueTelemetry();
            Serial.println("[CON] Telemetry broadcast queued");
        }

    } else if (strcmp(cmd, "auto") == 0) {
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            gState.autoMode = true;
            xSemaphoreGive(xStateMutex);
        }
        gConfig.boot_auto_mode = 1;
        saveConfig();
        Serial.println("[CON] Mode → AUTO (saved to NVS)");

    } else if (strcmp(cmd, "manual") == 0) {
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            gState.autoMode = false;
            xSemaphoreGive(xStateMutex);
        }
        gConfig.boot_auto_mode = 0;
        saveConfig();
        Serial.println("[CON] Mode → MANUAL (saved to NVS)");

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
            Serial.printf("[CON] Toggle %s injected (control engine applies on next tick)\n", cmd);
        } else {
            Serial.println("[CON] Must be in MANUAL mode — run 'manual' first");
        }

    } else if (strcmp(cmd, "config") == 0) {
        Serial.printf("[CON] Config (version %u):\n", CONFIG_VERSION);
        Serial.printf("[CON]   pump_min_runtime_ms       = %lu\n",  gConfig.pump_min_runtime_ms);
        Serial.printf("[CON]   pump_min_cooldown_ms      = %lu\n",  gConfig.pump_min_cooldown_ms);
        Serial.printf("[CON]   replenish_runon_ms        = %lu\n",  gConfig.replenish_runon_ms);
        Serial.printf("[CON]   telemetry_interval_ms     = %lu\n",  gConfig.telemetry_interval_ms);
        Serial.printf("[CON]   network_timeout_ms        = %lu\n",  gConfig.network_timeout_ms);
        Serial.printf("[CON]   ack_timeout_ms            = %lu\n",  gConfig.ack_timeout_ms);
        Serial.printf("[CON]   ack_max_retries           = %u\n",   gConfig.ack_max_retries);
        Serial.printf("[CON]   boot_auto_mode            = %u\n",   gConfig.boot_auto_mode);

    } else if (strcmp(cmd, "stats") == 0) {
        Serial.printf("[CON] Stats (boot #%u):\n", gStats.boot_count);
        Serial.printf("[CON]   runtime P1=%lu s  P2=%lu s  P3=%lu s  Pond=%lu s\n",
                      gStats.runtime_p1_s, gStats.runtime_p2_s,
                      gStats.runtime_p3_s, gStats.runtime_pond_s);
        Serial.printf("[CON]   fill_cycles=%u  last_fault=%u  uptime=%lu s\n",
                      gStats.fill_cycles, gStats.last_fault, millis() / 1000);

    } else if (strcmp(cmd, "reboot") == 0) {
        Serial.println("[CON] Rebooting...");
        Serial.flush();
        esp_restart();

    } else {
        Serial.printf("[CON] Unknown command: '%s'  (try 'help')\n", cmd);
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
                while (*p == ' ') p++;
                handleConsoleCommand(p);
            } else {
                buf[idx++] = c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

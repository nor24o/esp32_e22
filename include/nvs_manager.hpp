#pragma once

#include <Preferences.h>
#include "globals.hpp"

// =============================================================================
// PERSISTENT STORAGE — CONFIG AND STATS
//
// The ESP32 "Preferences" library writes key-value pairs to the internal flash.
// This is called NVS (Non-Volatile Storage).  Data survives reboots and power cuts.
//
// Two things are stored:
//   gConfig  — runtime tuning parameters (pump timings, intervals, etc.)
//   gStats   — accumulated counters (pump runtimes, fill cycles, boot count)
// =============================================================================

static NodeConfig    gConfig;   // Loaded from NVS in setup(), writable by gateway
static StatsPayload  gStats;    // Loaded from NVS in setup(), updated during operation

// ── Default config ────────────────────────────────────────────────────────────
// Applied on first boot or when CONFIG_VERSION changes.
static void resetConfigToDefaults() {
    gConfig.pump_min_runtime_ms     = DEF_PUMP_MIN_RUNTIME_MS;
    gConfig.pump_min_cooldown_ms    = DEF_PUMP_MIN_COOLDOWN_MS;
    gConfig.replenish_runon_ms      = DEF_REPLENISH_RUNON_MS;
    gConfig.telemetry_interval_ms   = DEF_TELEMETRY_INTERVAL_MS;
    gConfig.network_timeout_ms      = DEF_NETWORK_TIMEOUT_MS;
    gConfig.cmd_response_timeout_ms = DEF_CMD_RESPONSE_TIMEOUT_MS;
    gConfig.boot_auto_mode          = DEF_BOOT_AUTO_MODE;
    memset(gConfig._reserved, 0, sizeof(gConfig._reserved));
}

// ── Validation ────────────────────────────────────────────────────────────────
// Called before applying a config sent by the gateway.
// Rejects values that would make the system behave badly.
static bool validateConfig(const NodeConfig &c) {
    if (c.pump_min_runtime_ms     < 5000UL   || c.pump_min_runtime_ms     > 3600000UL) return false;
    if (c.pump_min_cooldown_ms    < 5000UL   || c.pump_min_cooldown_ms    > 3600000UL) return false;
    if (c.replenish_runon_ms      < 30000UL  || c.replenish_runon_ms      > 86400000UL) return false;
    if (c.telemetry_interval_ms   < 5000UL   || c.telemetry_interval_ms   > 3600000UL) return false;
    if (c.network_timeout_ms      < 10000UL  || c.network_timeout_ms      > 3600000UL) return false;
    if (c.cmd_response_timeout_ms < 5000UL   || c.cmd_response_timeout_ms > 120000UL)  return false;
    return true;
}

// ── Save / load ───────────────────────────────────────────────────────────────

static void saveConfig() {
    Preferences p;
    p.begin(NVS_NAMESPACE, false);           // false = read-write
    p.putUChar(NVS_CFG_VER_KEY, CONFIG_VERSION);
    p.putBytes(NVS_CFG_KEY, &gConfig, sizeof(gConfig));
    p.end();
}

// Only the mutable counters are saved here.
// boot_count is incremented and saved once in setup().
// uptime_s is computed from millis() at query time, never saved.
static void saveStats() {
    Preferences p;
    p.begin(NVS_NAMESPACE, false);
    p.putUInt("rt1", gStats.runtime_p1_s);
    p.putUInt("rt2", gStats.runtime_p2_s);
    p.putUInt("rt3", gStats.runtime_p3_s);
    p.putUInt("rtp", gStats.runtime_pond_s);
    p.putUShort("fc", gStats.fill_cycles);
    p.putUChar("lf",  gStats.last_fault);
    p.end();
}

static void loadConfig() {
    Preferences p;
    p.begin(NVS_NAMESPACE, true);   // true = read-only

    // Config blob — only load if the version matches and the size is correct.
    // If either check fails, reset to safe defaults so the device still works.
    uint8_t ver = p.getUChar(NVS_CFG_VER_KEY, 0);
    if (ver == CONFIG_VERSION && p.getBytesLength(NVS_CFG_KEY) == sizeof(gConfig)) {
        p.getBytes(NVS_CFG_KEY, &gConfig, sizeof(gConfig));
    } else {
        resetConfigToDefaults();
    }

    // Stats — each counter is stored individually so a corrupt config does not
    // wipe the stats, and vice versa.
    memset(&gStats, 0, sizeof(gStats));
    gStats.runtime_p1_s   = p.getUInt("rt1", 0);
    gStats.runtime_p2_s   = p.getUInt("rt2", 0);
    gStats.runtime_p3_s   = p.getUInt("rt3", 0);
    gStats.runtime_pond_s = p.getUInt("rtp", 0);
    gStats.fill_cycles    = p.getUShort("fc", 0);
    gStats.boot_count     = p.getUShort("bc", 0);
    gStats.last_fault     = p.getUChar("lf", 0);
    gStats.uptime_s       = 0;   // filled at query time

    p.end();
}

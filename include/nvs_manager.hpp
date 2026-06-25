#pragma once

#include <Preferences.h>
#include "globals.hpp"

// =============================================================================
// PERSISTENT CONFIG AND STATS
// =============================================================================

static NodeConfig    gConfig;
static StatsPayload  gStats;
static uint8_t       gRadioCrashStreak = 0;  // consecutive crash-boots during radio init

static void resetConfigToDefaults() {
    gConfig.pump_min_runtime_ms   = DEF_PUMP_MIN_RUNTIME_MS;
    gConfig.pump_min_cooldown_ms  = DEF_PUMP_MIN_COOLDOWN_MS;
    gConfig.replenish_runon_ms    = DEF_REPLENISH_RUNON_MS;
    gConfig.telemetry_interval_ms = DEF_TELEMETRY_INTERVAL_MS;
    gConfig.network_timeout_ms    = DEF_NETWORK_TIMEOUT_MS;
    gConfig.ack_timeout_ms        = DEF_ACK_TIMEOUT_MS;
    gConfig.overcurrent_thresh    = DEF_OVERCURRENT_THRESH;
    gConfig.dryrun_thresh         = DEF_DRYRUN_THRESH;
    gConfig.ack_max_retries          = DEF_ACK_MAX_RETRIES;
    gConfig.boot_auto_mode           = DEF_BOOT_AUTO_MODE;
    gConfig.overcurrent_grace_ticks  = DEF_OVERCURRENT_GRACE_TICKS;
    gConfig.fault_lockout_enabled    = DEF_FAULT_LOCKOUT_ENABLED;
}

// Sanity-check incoming config before applying it.
static bool validateConfig(const NodeConfig &c) {
    if (c.pump_min_runtime_ms   < 5000UL   || c.pump_min_runtime_ms   > 3600000UL) return false;
    if (c.pump_min_cooldown_ms  < 5000UL   || c.pump_min_cooldown_ms  > 3600000UL) return false;
    if (c.replenish_runon_ms    < 30000UL  || c.replenish_runon_ms    > 86400000UL) return false;
    if (c.telemetry_interval_ms < 5000UL   || c.telemetry_interval_ms > 3600000UL) return false;
    if (c.network_timeout_ms    < 10000UL  || c.network_timeout_ms    > 3600000UL) return false;
    if (c.ack_timeout_ms        < 1000UL   || c.ack_timeout_ms        > 60000UL)  return false;
    if (c.overcurrent_thresh    > 4095)                                            return false;
    if (c.dryrun_thresh         >= c.overcurrent_thresh)                           return false;
    if (c.ack_max_retries       < 1        || c.ack_max_retries       > 10)       return false;
    return true;
}

static void saveConfig() {
    Preferences p;
    p.begin(NVS_NAMESPACE, false);
    p.putUChar(NVS_CFG_VER_KEY, CONFIG_VERSION);
    p.putBytes(NVS_CFG_KEY, &gConfig, sizeof(gConfig));
    p.end();
}

// Saves all mutable stats fields except uptime_s and boot_count.
// boot_count is incremented and saved once in setup().
// uptime_s is ephemeral (set from millis() at query time).
static void saveStats() {
    Preferences p;
    p.begin(NVS_NAMESPACE, false);
    p.putUInt("rt1", gStats.runtime_p1_s);
    p.putUInt("rt2", gStats.runtime_p2_s);
    p.putUInt("rt3", gStats.runtime_p3_s);
    p.putUInt("rtp", gStats.runtime_pond_s);
    p.putUShort("fc",  gStats.fill_cycles);
    p.putUChar("lf",   gStats.last_fault);
    p.end();
}

static void loadConfig() {
    Preferences p;
    p.begin(NVS_NAMESPACE, true);  // read-only

    // Config blob
    uint8_t ver = p.getUChar(NVS_CFG_VER_KEY, 0);
    if (ver == CONFIG_VERSION && p.getBytesLength(NVS_CFG_KEY) == sizeof(gConfig)) {
        p.getBytes(NVS_CFG_KEY, &gConfig, sizeof(gConfig));
    } else {
        resetConfigToDefaults();
    }

    // Stats
    memset(&gStats, 0, sizeof(gStats));
    gStats.runtime_p1_s   = p.getUInt("rt1", 0);
    gStats.runtime_p2_s   = p.getUInt("rt2", 0);
    gStats.runtime_p3_s   = p.getUInt("rt3", 0);
    gStats.runtime_pond_s = p.getUInt("rtp", 0);
    gStats.fill_cycles     = p.getUShort("fc", 0);
    gStats.boot_count      = p.getUShort("bc", 0);
    gStats.last_fault      = p.getUChar("lf", 0);
    gStats.uptime_s        = 0;  // computed at query time
    gRadioCrashStreak = p.getUChar(NVS_RADIO_STREAK_KEY, 0);
    p.end();
}

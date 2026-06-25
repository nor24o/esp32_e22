#pragma once

#include "globals.hpp"
#include "nvs_manager.hpp"

// =============================================================================
// PUMP HYSTERESIS MANAGEMENT
// =============================================================================

struct PumpHysteresis {
    bool     state;
    uint32_t lastOnTime_ms;
    uint32_t lastOffTime_ms;
    bool     initialized;
};

// These read gConfig directly — only called from ControlEngine (no mutex needed).
static bool canTurnPumpOn(const PumpHysteresis &ph) {
    if (!ph.initialized) return true;
    if (ph.state)         return false;
    return (millis() - ph.lastOffTime_ms) >= gConfig.pump_min_cooldown_ms;
}

static bool canTurnPumpOff(const PumpHysteresis &ph) {
    if (!ph.initialized) return true;
    if (!ph.state)        return false;
    return (millis() - ph.lastOnTime_ms) >= gConfig.pump_min_runtime_ms;
}

// relay_pin: GPIO for relay output (active-low); -1 = no local relay (pond, remote only)
static void setPumpState(PumpHysteresis &ph, bool newState, int8_t relay_pin) {
    if (ph.initialized && (newState == ph.state)) return;
    if (relay_pin >= 0) {
        digitalWrite((uint8_t)relay_pin, newState ? LOW : HIGH);  // active-low
    }
    if (newState) ph.lastOnTime_ms  = millis();
    else          ph.lastOffTime_ms = millis();
    ph.state       = newState;
    ph.initialized = true;
}

// Run-time tracking helpers – called from ControlEngine alongside setPumpState.
// onSince == 0 means pump was not tracked as running.
static inline void trackPumpOn(uint32_t &onSince) {
    if (onSince == 0) onSince = millis();
}

static inline uint32_t trackPumpOff(uint32_t &onSince) {
    if (onSince == 0) return 0;
    uint32_t elapsed = (millis() - onSince) / 1000UL;
    onSince = 0;
    return elapsed;
}

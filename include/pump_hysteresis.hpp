#pragma once

#include "globals.hpp"
#include "nvs_manager.hpp"

// =============================================================================
// PUMP HYSTERESIS
//
// "Hysteresis" means we enforce minimum ON and OFF times so the pump is not
// rapidly switched on and off, which would wear out the motor and relay contacts.
//
//   pump_min_runtime_ms  — once a pump starts, it must run for at least this long
//   pump_min_cooldown_ms — once a pump stops, it must rest for at least this long
//
// Each pump gets its own PumpHysteresis struct to track its individual timers.
// =============================================================================

struct PumpHysteresis {
    bool     state;           // true = pump is currently ON
    uint32_t lastOnTime_ms;   // millis() when the pump was last turned ON
    uint32_t lastOffTime_ms;  // millis() when the pump was last turned OFF
    bool     initialized;     // false until the pump has been turned on or off at least once
};

// Returns true if it is safe to turn this pump ON right now.
// Reads from gConfig directly — only call from ControlEngine (no mutex needed).
static bool canTurnPumpOn(const PumpHysteresis &ph) {
    if (!ph.initialized) return true;   // first use — no restrictions yet
    if (ph.state)        return false;  // already ON
    // Must have been OFF for at least pump_min_cooldown_ms
    return (millis() - ph.lastOffTime_ms) >= gConfig.pump_min_cooldown_ms;
}

// Returns true if it is safe to turn this pump OFF right now.
static bool canTurnPumpOff(const PumpHysteresis &ph) {
    if (!ph.initialized) return true;
    if (!ph.state)       return false;  // already OFF
    // Must have been ON for at least pump_min_runtime_ms
    return (millis() - ph.lastOnTime_ms) >= gConfig.pump_min_runtime_ms;
}

// Applies a new pump state.
//   relay_pin — the GPIO number for the relay (active-low).
//               Pass -1 for the pond pump (no local relay; controlled remotely).
static void setPumpState(PumpHysteresis &ph, bool newState, int8_t relay_pin) {
    if (ph.initialized && (newState == ph.state)) return;  // no change
    if (relay_pin >= 0) {
        // Active-low relay: LOW = energise coil = pump ON
        digitalWrite((uint8_t)relay_pin, newState ? LOW : HIGH);
    }
    if (newState) ph.lastOnTime_ms  = millis();
    else          ph.lastOffTime_ms = millis();
    ph.state       = newState;
    ph.initialized = true;
}

// Call this when a pump turns ON.  Records the start time so we can calculate
// how long it ran when it eventually turns off.
static inline void trackPumpOn(uint32_t &onSince) {
    onSince = millis();
}

// Call this when a pump turns OFF.  Returns how many seconds the pump ran.
// The returned value should be added to the matching gStats.runtime_xxx_s counter.
static inline uint32_t trackPumpOff(uint32_t &onSince) {
    if (onSince == 0) return 0;
    uint32_t elapsed = (millis() - onSince) / 1000UL;
    onSince = 0;
    return elapsed;
}

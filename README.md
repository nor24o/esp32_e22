# ESP32 Tank Water Level Controller — Node `0x02`

> LoRa-networked pump controller · ESP32 / ESP32-S3 + SX1262 E22 · FreeRTOS · PCF8574 I/O expander · 3 local pumps + 1 remote pond pump

---

## Repository Branches

| Branch | Target | Purpose |
|--------|--------|---------|
| `tank-controller` | ESP32 / ESP32-S3 | Tank controller firmware (this device) |
| `lora_com_test`   | TTGO LoRa32 V1.0 | Standalone test sketch — simulates gateway and pond node, includes web dashboard |

---

## Network Topology

```
              ┌─────────────────┐
              │  Gateway  0x01  │  (PC / TTGO test node / home automation hub)
              └────────┬────────┘
                       │  LoRa 868 MHz
           ┌───────────┴────────────┐
           │                        │
  ┌────────▼────────┐     ┌─────────▼───────┐
  │  TANK    0x02   │◄───►│  POND    0x03   │
  │  (this device)  │     │  remote pump    │
  └─────────────────┘     └─────────────────┘
```

| Node | Address | Role |
|------|:-------:|------|
| Gateway | `0x01` | Central coordinator / home automation |
| **Tank Controller** | **`0x02`** | **This device** — controls local pumps, monitors water level |
| Pond Remote | `0x03` | Remote pond pump node |

---

## Hardware — Tank Controller Node (0x02)

### Board

Primary target: **ESP32-S3 DevKitC-1** (`esp32s3dev` environment)
Fallback target: **ESP32 DevKit** (`esp32dev` environment)

Most GPIO assignments differ between the two — see per-target tables below.

---

### LoRa Radio — EBYTE E22-900M22S (SX1262)

Connected via hardware SPI. The SX1262 uses **DIO1** for the RxDone / TxDone interrupt (unlike the SX1276 which uses DIO0).

| Signal | ESP32-S3 GPIO | ESP32 GPIO | Notes |
|--------|:------------:|:---------:|-------|
| NSS (CS) | 5 | 5 | Chip select |
| SCK | 18 | 18 | SPI clock |
| MOSI | 11 | 23 | |
| MISO | 13 | 19 | |
| DIO1 | 16 | 26 | RxDone / TxDone interrupt |
| BUSY | 17 | 27 | Module busy signal |
| NRST | 14 | 14 | Hardware reset |

**RF settings (must match all other nodes exactly):**

| Parameter | Value |
|-----------|-------|
| Frequency | 868.0 MHz (EU ISM band) |
| Bandwidth | 125 kHz |
| Spreading Factor | 9 |
| Coding Rate | 4/5 |
| Sync Word | `0x12` |
| TX Power | 22 dBm |
| Preamble | 8 symbols |
| Packet airtime | ~330 ms for 38 bytes at SF9 |

---

### I2C Bus — PCF8574 I/O Expander (`0x27`)

All buttons and float switches are connected through a **PCF8574 8-bit I/O expander** instead of native ESP32 GPIOs. The I2C bus is shared between the PCF8574 and can accommodate additional peripherals.

| Signal | GPIO (both targets) |
|--------|:-------------------:|
| SDA | 1 |
| SCL | 2 |

**PCF8574 I2C address:** `0x27` (A0=A1=A2 tied HIGH or to VCC)

All 8 pins are configured as quasi-bidirectional inputs (write `0xFF` to the PCF). External signals pull individual pins HIGH or LOW:

#### P0–P2 · Vertical Float Switches (water level sensors)

Mechanical reed float switches. **Active-LOW** — when the float rises (water present at that level), the reed closes and pulls the PCF pin LOW.

| PCF Pin | Level | Meaning |
|:-------:|:-----:|---------|
| P0 | 1 | Lowest float — water present at bottom third |
| P1 | 2 | Middle float — water at mid level |
| P2 | 3 | Upper float — tank full |

**Idle state (no water):** all three pins HIGH (open circuit, pulled up by PCF quasi-bidirectional output).
**Tank full:** P0 + P1 + P2 all LOW.
**Water level 0–3** is derived by counting how many switches are closed.

> Wire each float switch between the PCF pin and GND. No external pull-up resistor needed — the PCF's internal quasi-bidirectional pull-up (~100 µA) is sufficient.

#### P3–P7 · TTP223 Capacitive Touch Modules (buttons)

**TTP223 default wiring** (A-pad open, B-pad open): output is **active-HIGH** — idle LOW, touched HIGH. Rising edge (LOW → HIGH) = touch event.

> If your TTP223 modules have the **A-pad bridged**, they operate active-LOW (idle HIGH, touched LOW). In that case set `#define TTP223_TOUCHED_LEVEL 0` in `config.hpp` and the firmware will switch to falling-edge detection.

| PCF Pin | `config.hpp` define | Function |
|:-------:|---------------------|---------|
| P3 | `PCF_TOUCH_MODE` | Toggle AUTO / MANUAL mode |
| P4 | `PCF_TOUCH_P1` | Pump 1 manual toggle |
| P5 | `PCF_TOUCH_P2` | Pump 2 manual toggle |
| P6 | `PCF_TOUCH_P3` | Pump 3 manual toggle |
| P7 | `PCF_TOUCH_POND` | Pond pump manual toggle |

**Touch module wiring:** connect TTP223 `VCC` to 3.3 V, `GND` to GND, `OUT` to the corresponding PCF pin. The PCF's quasi-bidirectional pull-up keeps the pin LOW when the TTP223 output is idle (low-impedance LOW), and the TTP223 drives it HIGH on touch.

---

### WS2812B LED Strip

| Parameter | Value |
|-----------|-------|
| Data pin | GPIO **48** (ESP32-S3) |
| LED count | 10 |
| Color order | GRB |
| Library | FastLED |

> GPIO 48 must be used only on ESP32-S3. For classic ESP32 this pin does not exist — if porting, reassign to GPIO 12 or another available output.

**LED layout:**

```
LED:  [ 0 ][ 1 ][ 2 ][ 3 ][ 4 ][ 5 ][ 6 ][ 7 ][ 8 ][ 9 ]
       ─────────────────────  ────  ────  ────  ────  ────
       Water level (0-2)      Stat  Mode  P1    P2    P3   Pond
```

**Water level LEDs (0–2):**

```
  Empty  ○ ○ ○   Level 0 — triggers pond pump in auto mode
  Low    ● ○ ○   Level 1
  Mid    ● ● ○   Level 2
  Full   ● ● ●   Level 3 — stops pond pump in auto mode
  ● = AQUA    ○ = off
```

**Status LED (LED 3) — priority order (highest first):**

| Colour | Pattern | Meaning |
|--------|---------|---------|
| RED | Solid | Fault lockout (overcurrent or dry-run) |
| RED | 400 ms flash | Comms error — gateway or pond timeout |
| PURPLE | 200 ms flash | LoRa packet received |
| CYAN | 200 ms flash | LoRa packet transmitted |
| YELLOW | Pulse | Awaiting ACK from pond node |
| GREEN | Solid | Auto mode — all OK |
| BLUE | Solid | Manual mode — all OK |

**Mode and pump LEDs (LEDs 4–8, LED 9):**

| LED | ON colour | OFF colour | Meaning |
|:---:|-----------|------------|---------|
| 4 | GREEN | BLUE (dim) | Auto mode / Manual mode |
| 5 | GREEN | RED (dim) | Pump P1 running / off |
| 6 | GREEN | RED (dim) | Pump P2 running / off |
| 7 | GREEN | RED (dim) | Pump P3 running / off |
| 8 | CYAN | BLUE (dim) | Pond pump running / off |

---

### Relay Outputs

Active-LOW outputs (LOW = relay energised = pump ON).

| Signal | GPIO (ESP32-S3) | GPIO (ESP32) | Notes |
|--------|:--------------:|:-----------:|-------|
| Relay P1 | 47 | 13 | Pump 1 |
| Relay P2 | 41 | 15 | Pump 2 |
| Relay P3 | 39 | 17 | Pump 3 |
| Buzzer | 21 | 21 | Optional piezo buzzer |

> Relays are driven through a transistor or relay driver IC. Do not connect relay coils directly to ESP32 GPIOs.

---

### Current Sensing (ADC)

One ADC input per pump monitors the motor current via an inline current-sense resistor or hall-effect sensor. Raw 12-bit ADC values (0–4095) are compared against configurable thresholds.

| Signal | ESP32-S3 GPIO | ESP32 GPIO | ADC channel |
|--------|:------------:|:---------:|-------------|
| Current P1 | 4 | 4 | ADC1_CH3 (S3) / ADC2_CH0 (ESP32) |
| Current P2 | 7 | 39 | ADC1_CH6 (S3) / ADC1_CH3 (ESP32, input-only) |
| Current P3 | 3 | 2 | ADC1_CH2 (S3) / ADC2_CH2 (ESP32) |

> ESP32 GPIO 39 is input-only and has no internal pull-up/down — safe as ADC input.
> Each reading is averaged over 8 samples (`ADC_SAMPLES 8`).

**Threshold defaults (NVS, adjustable at runtime):**

| Parameter | Default | Meaning |
|-----------|:-------:|---------|
| `overcurrent_thresh` | 3200 | ADC raw ≥ this → overcurrent |
| `dryrun_thresh` | 150 | ADC raw ≤ this while pump ON → dry-run |
| `overcurrent_grace_ticks` | 5 | Must exceed threshold for 5 × 50 ms = 250 ms before fault trips (filters inrush) |

---

## Hardware — TTGO LoRa32 V1.0 Test Node (`lora_com_test` branch)

Used to test communication with the tank controller without needing a full gateway or pond node deployment. Can simulate both **GATEWAY (0x01)** and **POND NODE (0x03)** at runtime.

### Board

**TTGO LoRa32 V1.0** — ESP32 Xtensa dual-core + SX1276 (HPD13A module, 868 MHz)

> The SX1276 uses **DIO0** for RxDone, not DIO1 like the SX1262. RadioLib handles this difference transparently through the `SX1276` class.

### LoRa Radio — SX1276 (HPD13A on TTGO board)

| Signal | GPIO | Notes |
|--------|:----:|-------|
| NSS (CS) | 18 | |
| SCK | 5 | |
| MOSI | 27 | |
| MISO | 19 | |
| DIO0 | 26 | RxDone / TxDone interrupt |
| RST | 14 | Hardware reset |
| DIO1 | — | Not used (SX1276 RxDone is on DIO0) |

RF settings are identical to the tank controller (868 MHz / BW 125 / SF9 / CR 4/5 / Sync `0x12`).

### Web Dashboard

The TTGO test sketch creates a WiFi access point and serves a real-time web dashboard via WebSocket.

| Item | Value |
|------|-------|
| WiFi SSID | `LoRaTest-TTGO` |
| WiFi password | `loratest1` |
| URL | `http://192.168.4.1` |
| WebSocket | `ws://192.168.4.1/ws` |

Dashboard features: animated tank fill, pump state dots, live RSSI/SNR, role switcher (GATEWAY / POND), pump command buttons, Config and Stats tables, colour-coded event log.

---

## Project Structure

```
esp32_e22/
├── platformio.ini          — three build environments
├── src/
│   ├── tank/
│   │   └── main.cpp        — setup() + loop() only; includes all hpp files
│   └── ttgo_test/
│       └── lora_test_gateway.cpp  — standalone TTGO dual-role sketch
└── include/
    ├── config.hpp           — pin definitions, RF settings, NVS keys, defaults
    ├── protocol.hpp         — packed network structs, MessageType enum
    ├── globals.hpp          — SystemState, FreeRTOS handles, peripheral instances
    ├── nvs_manager.hpp      — gConfig, gStats, load/save helpers
    ├── helpers.hpp          — telemetry builder, sendPumpCommand, sendAck
    ├── pump_hysteresis.hpp  — min-runtime + cooldown logic per pump
    ├── task_input_sensor.hpp    — Core 1 P3: PCF8574 poll, float switches, touch
    ├── task_lora_transceiver.hpp — Core 0 P2: TX queue drain, RX ISR + readback
    ├── task_control_engine.hpp  — Core 1 P4: relay logic, auto/manual, ACK retry
    ├── task_ui_animation.hpp    — Core 0 P1: WS2812B LED strip effects
    └── web_page.h           — PROGMEM HTML/CSS/JS dashboard (TTGO sketch only)
```

### Build Environments

| Environment | Board | Source dir | Use for |
|-------------|-------|:---------:|---------|
| `esp32dev` | ESP32 DevKit | `src/tank/` | Classic ESP32 Xtensa |
| `esp32s3dev` | ESP32-S3 DevKitC-1 | `src/tank/` | ESP32-S3 (primary target) |
| `ttgo_lora_test` | TTGO LoRa32 V1.0 | `src/ttgo_test/` | Communication test + web UI |

Each environment uses `build_src_filter = +<subdir/>` so only its own entry point compiles — merging branches will never cause duplicate `setup()`/`loop()` linker errors.

---

## FreeRTOS Task Architecture

```
  Core 0                              Core 1
  ┌───────────────────────┐           ┌───────────────────────┐
  │  LoRaTransceiver      │           │  ControlEngine        │
  │  Priority 2           │           │  Priority 4  50 ms    │
  │  Event-driven         │           │  Relay / pump logic   │
  │  TX queue drain       │           │  OC grace counter     │
  │  DIO1 ISR → RX queue  │           │  ACK retry logic      │
  └──────────┬────────────┘           │  Telemetry timer      │
             │  RxQueue               └──────────┬────────────┘
  ┌──────────▼────────────┐                      │  StateMutex
  │  UIAnimation          │           ┌──────────▼────────────┐
  │  Priority 1  30 ms    │           │  InputSensorPoll      │
  │  WS2812B 10-LED strip │           │  Priority 3  20 ms    │
  └───────────────────────┘           │  PCF8574 I2C poll     │
                                      │  Float sw + touch     │
                                      │  ADC current (×8)     │
                                      └───────────────────────┘

  Shared resources:
    StateMutex       — guards SystemState struct across both cores
    LoRaIrqSemaphore — DIO1 ISR → LoRaTransceiver task
    TxQueue (×8)     — any task → LoRaTransceiver
    RxQueue (×8)     — LoRaTransceiver → ControlEngine
    I2cMutex         — guards Wire bus (PCF8574 access from InputSensorPoll)
```

---

## LoRa Packet Protocol

### Packet Layout (38 bytes)

```
 Byte  0    1         2          3         4        5         6 … 37
      ┌────────────┬──────────┬──────────┬────────┬─────────┬────────────────┐
      │ magic 5A6B │ target_id│ sender_id│ msg_id │msg_type │    payload     │
      └────────────┴──────────┴──────────┴────────┴─────────┴────────────────┘
       ◄────────── Header: 6 bytes ──────────────►  ◄── Payload union: 32 B ──►
```

### Message Types

| ID | Name | Direction | Description |
|:--:|------|-----------|-------------|
| 1 | `MSG_TELEMETRY` | Node → Gateway | Periodic sensor data |
| 2 | `MSG_COMMAND` | Gateway → Node | Pump on/off command |
| 3 | `MSG_ACK` | Node ↔ Node | Acknowledge receipt |
| 4 | `MSG_CONFIG_GET` | Gateway → Node | Request current config |
| 5 | `MSG_CONFIG_SET` | Gateway → Node | Write config to NVS |
| 6 | `MSG_CONFIG_RESP` | Node → Gateway | Config echo (verify applied) |
| 7 | `MSG_STATS_GET` | Gateway → Node | Request operational stats |
| 8 | `MSG_STATS_RESP` | Node → Gateway | Pump runtimes, faults, etc. |

### Telemetry Payload (8 bytes)

```c
struct TelemetryData {
    uint8_t  water_level;   // 0-3 (count of float switches closed)
    uint8_t  system_flags;  // bit0=Auto  bit1=P1  bit2=P2  bit3=P3  bit4=Pond
    int16_t  temperature;   // °C × 10  (0 = sensor not fitted)
    uint8_t  humidity;      // % RH      (0 = sensor not fitted)
    uint8_t  error_code;    // 0=OK  1=Overcurrent  2=DryRun  3=Comms
    int8_t   last_rssi;     // dBm of last received packet (as seen by this node)
    int8_t   last_snr;      // SNR in dB
};
```

---

## Runtime Configuration (NVS)

All parameters stored in NVS namespace `tanknode`, tunable at runtime over LoRa via `MSG_CONFIG_SET` without reflashing. Applied immediately and survive reboots.

| Parameter | Default | Range | Description |
|-----------|:-------:|-------|-------------|
| `pump_min_runtime_ms` | 30 000 | 5 000 – 3 600 000 | Minimum ON time per pump cycle |
| `pump_min_cooldown_ms` | 60 000 | 5 000 – 3 600 000 | Minimum OFF time before restart |
| `replenish_runon_ms` | 300 000 | 30 000 – 86 400 000 | Minimum pond pump run time (fill) |
| `telemetry_interval_ms` | 30 000 | 5 000 – 3 600 000 | How often to send telemetry |
| `network_timeout_ms` | 60 000 | 10 000 – 3 600 000 | No contact → comms error |
| `ack_timeout_ms` | 10 000 | 1 000 – 60 000 | Wait for ACK before retry |
| `overcurrent_thresh` | 3 200 | 0 – 4 095 | ADC raw cutoff (12-bit) |
| `dryrun_thresh` | 150 | 0 – overcurrent | ADC raw floor |
| `ack_max_retries` | 5 | 1 – 10 | Retries before comms error |
| `boot_auto_mode` | 1 | 0 or 1 | Startup mode (1=auto) |
| `overcurrent_grace_ticks` | 5 | 0 – 255 | Ticks × 50 ms before fault trips |
| `fault_lockout_enabled` | 1 | 0 or 1 | 1=kill relays on fault  0=warn only |

---

## Persistent Statistics (NVS)

Queryable at runtime via `MSG_STATS_GET`. Survive reboots.

| Field | Description |
|-------|-------------|
| `uptime_s` | Total uptime since last boot (seconds) |
| `runtime_p1_s` | Pump P1 total ON time |
| `runtime_p2_s` | Pump P2 total ON time |
| `runtime_p3_s` | Pump P3 total ON time |
| `runtime_pond_s` | Pond pump total ON time |
| `fill_cycles` | Tank refill cycles — high count may indicate a leak |
| `boot_count` | Total reboots |
| `last_fault` | Last fault code (1=overcurrent, 2=dry-run) |

---

## Fault Handling

### Overcurrent Protection

Each pump's current is sampled 8× at 50 ms intervals. The ADC value must **continuously** exceed `overcurrent_thresh` for `overcurrent_grace_ticks` ticks (default 250 ms) before any action is taken — short inrush spikes during motor start are ignored.

### Dry-Run Protection

Same grace period. If the ADC reads **below** `dryrun_thresh` while a pump is ON, the pump is assumed to be running dry (no water load).

### Fault Modes

| `fault_lockout_enabled` | Behaviour on fault |
|:-----------------------:|--------------------|
| `1` (default) | Kill all relays immediately → `errorCode` set → saved to NVS → LED turns solid RED. **Clear by pressing MODE touch button.** |
| `0` (warn-only) | `errorCode` set and LED shows fault colour, but relays keep running. Clears automatically when current returns to normal. Useful during commissioning. |

> Comms error (`errorCode = 3`) never triggers a relay lockout in either mode.

---

## Building & Flashing

### Dependencies (auto-managed by PlatformIO)

```ini
# Tank controller
jgromes/RadioLib @ ^6.6.0
fastled/FastLED @ ^3.6.0
adafruit/DHT sensor library @ ^1.4.6
adafruit/Adafruit Unified Sensor @ ^1.1.14

# TTGO test sketch only
me-no-dev/ESPAsyncWebServer @ ^1.2.3
me-no-dev/AsyncTCP @ ^1.1.1
```

### Commands

```bash
# ── Tank Controller (ESP32-S3) ──────────────────────────────────
pio run -e esp32s3dev                          # build
pio run -e esp32s3dev -t upload                # flash
pio device monitor -e esp32s3dev               # serial monitor

# ── Tank Controller (classic ESP32) ────────────────────────────
pio run -e esp32dev
pio run -e esp32dev -t upload
pio device monitor -e esp32dev

# ── TTGO LoRa32 V1.0 test sketch ───────────────────────────────
pio run -e ttgo_lora_test
pio run -e ttgo_lora_test -t upload
pio device monitor -e ttgo_lora_test
# Then connect to WiFi "LoRaTest-TTGO" / "loratest1"
# and open http://192.168.4.1
```

### Expected Boot Output (tank controller)

```
[BOOT] Tank Controller Node 0x02 starting...
[BOOT] Boot #N  |  auto=1  |  fill cycles=M
[BOOT] Radio OK  |  Packet size: 38 bytes
[BOOT] All tasks created.  System operational.
```

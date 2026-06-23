# ESP32 E22 — Tank Water Level Controller Node (0x02)

A production-grade, LoRa-networked water level management system running on ESP32 + SX1262. Manages up to 3 local pumps and 1 remote pond pump with automatic level-based control, fault protection, and runtime configuration over LoRa.

---

## Network Topology

```
[ Gateway 0x01 ]
       |
   LoRa 433 MHz
       |
[ Tank Node 0x02 ]  <--LoRa-->  [ Pond Node 0x03 ]
    (this device)
```

| Node | Address | Role |
|------|---------|------|
| Gateway | 0x01 | Central coordinator |
| Tank Controller | 0x02 | **This device** |
| Pond Remote | 0x03 | Remote pump node |

---

## Hardware

**MCU:** ESP32 (Xtensa LX6 dual-core, Arduino + FreeRTOS)

| Component | Part | Interface |
|-----------|------|-----------|
| LoRa Radio | SX1262 (E22) | SPI |
| LED Strip | WS2812B × 10 | GPIO 12 |
| Temp/Humidity | DHT22 | GPIO 16 |
| Float Switches | × 4 | GPIO 34–39 |
| Buttons | × 5 | GPIO 32, 33, 25, 21, 22 |
| Relay Outputs | × 3 (P1–P3) | GPIO 13, 15, 2 |
| Current Sensors | × 3 (ADC) | GPIO 4, 0, 3 |

### GPIO Map

```
SPI (LoRa):  NSS=5   DIO1=26  NRST=14  BUSY=27
             SCK=18  MISO=19  MOSI=23

Outputs:     Relay P1=13   Relay P2=15  Relay P3=2
             LED Strip=12

Inputs:      Button MODE=32  P1=33  P2=25  P3=21  POND=22
             Float 1=34  Float 2=35  Float 3=36  Float 4=39
             DHT22=16

ADC:         Current P1=4   Current P2=0   Current P3=3
```

> **Hardware constraints:**
> - GPIO 34–39 are input-only — require 10 kΩ pull-ups to 3.3 V
> - GPIO 0 (Current P2 ADC) is a strapping pin — keep < 0.6 V at power-on
> - GPIO 3 (Current P3 ADC) shares UART0-RX — disable `Serial` in production builds

---

## Features

- **Auto mode** — monitors 4-level float switches and triggers pond pump with remote ACK handshake when tank is low
- **Manual mode** — all pumps controlled by local buttons
- **Pump hysteresis** — configurable minimum runtime and cooldown per pump
- **Overcurrent protection** — immediate relay cutoff when ADC exceeds threshold
- **Dry-run detection** — shuts down pump if no current draw is detected
- **Replay attack prevention** — signed int8 sequence tracking per sender
- **ACK handshake** — remote pond pump commands require LoRa acknowledgement with configurable retries
- **Network health monitoring** — tracks last contact from gateway and pond node; sets error flag on timeout
- **10-LED RGB feedback** — real-time water level, pump state, network status, and fault indication
- **Runtime config over LoRa** — gateway can query and update all node settings without reflashing
- **NVS persistence** — config, stats, and mode preference survive reboots

---

## LoRa Packet Protocol (v1.1)

### Packet Layout

```
Total: 38 bytes

Header (6 bytes):
  magic_word  [2B]  — 0x5A6B (network identifier)
  target_id   [1B]  — destination node address
  sender_id   [1B]  — source node address
  msg_id      [1B]  — rolling message counter (replay protection)
  msg_type    [1B]  — message type (see below)

Payload union (32 bytes):
  TelemetryData | CommandData | NodeConfig | StatsPayload
```

### Message Types

| Type | Name | Direction | Description |
|------|------|-----------|-------------|
| 1 | `MSG_TELEMETRY` | Node → Gateway | Periodic sensor data |
| 2 | `MSG_COMMAND` | Gateway → Node | Pump on/off command |
| 3 | `MSG_ACK` | Node ↔ Node | Acknowledge receipt |
| 4 | `MSG_CONFIG_GET` | Gateway → Node | Request current config |
| 5 | `MSG_CONFIG_SET` | Gateway → Node | Write config to NVS |
| 6 | `MSG_CONFIG_RESP` | Node → Gateway | Config response |
| 7 | `MSG_STATS_GET` | Gateway → Node | Request operational stats |
| 8 | `MSG_STATS_RESP` | Node → Gateway | Stats response |

### Telemetry Payload

```c
struct TelemetryData {
    uint8_t  water_level;    // 0–4 (float switch count)
    uint8_t  system_flags;   // pump states, mode, fault
    float    temperature;    // °C (DHT22)
    float    humidity;       // % RH (DHT22)
    uint8_t  error_code;     // 0=ok, 1=overcurrent, 2=dry-run, 3=comms
    int16_t  rssi;           // last received packet RSSI
    float    snr;            // last received packet SNR
};
```

### RF Configuration

| Parameter | Value |
|-----------|-------|
| Frequency | 433.0 MHz |
| Bandwidth | 125 kHz |
| Spreading Factor | SF9 |
| Coding Rate | 4/5 |
| Sync Word | 0x12 |
| TX Power | 22 dBm |
| Preamble | 8 symbols |

> At SF9/125 kHz a full 38-byte packet takes ~330 ms to transmit (RadioLib `transmit()` blocks).

---

## Runtime Configuration

All parameters are stored in NVS and can be updated at runtime via `MSG_CONFIG_SET`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `pump_min_runtime_ms` | 30 000 | Minimum pump ON duration (ms) |
| `pump_min_cooldown_ms` | 60 000 | Minimum OFF time before restart (ms) |
| `replenish_runon_ms` | 300 000 | Minimum pond pump run for tank fill (ms) |
| `telemetry_interval_ms` | 30 000 | Telemetry send period (ms) |
| `network_timeout_ms` | 60 000 | No-contact timeout before comms error (ms) |
| `ack_timeout_ms` | 5 000 | Wait for ACK before retry (ms) |
| `overcurrent_thresh` | 3200 | 12-bit ADC raw cutoff (overcurrent) |
| `dryrun_thresh` | 150 | 12-bit ADC raw floor (dry-run detection) |
| `ack_max_retries` | 3 | Retries before marking comms failed |
| `boot_auto_mode` | 1 | Startup mode: 1 = auto, 0 = manual |

---

## Persistent Statistics

Accumulated in NVS across reboots, readable via `MSG_STATS_GET`:

| Field | Description |
|-------|-------------|
| `runtime_p1_s` | Total P1 run time (seconds) |
| `runtime_p2_s` | Total P2 run time (seconds) |
| `runtime_p3_s` | Total P3 run time (seconds) |
| `runtime_pond_s` | Total pond pump run time (seconds) |
| `fill_cycles` | Number of tank refill cycles (leak indicator) |
| `boot_count` | Total reboots |
| `last_fault` | Most recent fault code (1=overcurrent, 2=dry-run) |

---

## FreeRTOS Task Architecture

| Task | Core | Priority | Period | Duties |
|------|------|----------|--------|---------|
| `InputSensorPoll` | 1 | 3 | 20 ms | Debounce float switches & buttons; ADC averaging (8 samples); DHT22 poll |
| `ControlEngine` | 1 | 2 | 50 ms | State machine; pump hysteresis; drain RX queue; telemetry timer |
| `LoRaTransceiver` | 0 | 4 | Event | Drain TX queue; handle RX ISR; validate/dispatch packets |
| `UIAnimation` | 0 | 1 | 30 ms | LED updates: water level bar, pump indicators, network/fault status |

**Synchronization:**
- `SemaphoreHandle_t stateMutex` — guards `SystemState` struct
- Binary semaphore — LoRa RX interrupt → `LoRaTransceiver` task
- TX queue (depth 8) — other tasks → `LoRaTransceiver`
- RX queue (depth 8) — `LoRaTransceiver` → `ControlEngine`

---

## Operating Modes

### Auto Mode
1. `InputSensorPoll` reads 4 float switches → water level 0–4
2. When level = 0, `ControlEngine` sends `MSG_COMMAND` (pump ON) to Pond node (0x03)
3. Waits for `MSG_ACK`; retries up to `ack_max_retries` times
4. Pond pump runs for at least `replenish_runon_ms`
5. Stops when level = 4 AND minimum runtime elapsed AND ACK received from pond
6. Mode preference persisted to NVS

### Manual Mode
- Buttons toggle each pump independently
- Same hysteresis, ACK, and fault logic applies

### Fault Lockout
- Triggered by overcurrent (`ADC > overcurrent_thresh`) or dry-run (`ADC < dryrun_thresh`)
- All relays cut immediately; fault code written to NVS
- Press **MODE button** to clear lockout
- Error code 3 (comms) does **not** trigger lockout — pumps continue operating locally

---

## Building & Flashing

### Prerequisites
- [PlatformIO](https://platformio.org/) CLI or IDE extension

### Dependencies (auto-managed)

```ini
RadioLib          @ ^6.6.0
FastLED           @ ^3.6.0
Bounce2           @ ^2.71.0
DHT sensor library @ ^1.4.6
Adafruit Unified Sensor @ ^1.1.14
```

### Commands

```bash
# Build
platformio run -e esp32dev

# Flash
platformio run -e esp32dev --target upload

# Serial monitor (115200 baud)
platformio device monitor -p /dev/ttyUSB0 -b 115200
```

### Expected Boot Output

```
[BOOT] Tank Controller Node 0x02 v1.1 starting...
[BOOT] Boot #N  |  auto=1  |  fill cycles=M
[BOOT] Radio OK  |  Packet size: 38 bytes
[BOOT] All tasks created.  System operational.
```

---

## Project Structure

```
esp32_e22/
├── platformio.ini        # Build config (esp32dev, dependencies)
└── src/
    └── main.cpp          # Full firmware (~51 KB, 17 sections)
        ├── §1  Pin definitions
        ├── §2  Compile-time constants & RF settings
        ├── §3  Packed network protocol structs
        ├── §4  Global system state
        ├── §5  FreeRTOS handles
        ├── §6  Peripheral instances (SPI, Radio, LEDs, DHT, Bounce2)
        ├── §7  Replay-attack sequence tracking
        ├── §8  RadioLib DIO1 ISR
        ├── §9  NVS load/save
        ├── §10 Shared helpers (telemetry builder, pump commands, ACKs)
        ├── §11 Pump hysteresis management
        ├── §12 Task: InputSensorPoll
        ├── §13 Task: ControlEngine
        ├── §14 Task: LoRaTransceiver
        ├── §15 Task: UIAnimation
        ├── §16 setup()
        └── §17 loop() — intentionally empty
```

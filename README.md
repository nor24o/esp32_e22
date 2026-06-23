# ESP32 E22 — Tank Water Level Controller Node `0x02`

> LoRa-networked pump controller · ESP32 + SX1262 · FreeRTOS · 3 local pumps + 1 remote pond pump

---

## Network Topology

```
              ┌─────────────────┐
              │  Gateway  0x01  │
              └────────┬────────┘
                       │  LoRa 868 MHz
           ┌───────────┴────────────┐
           │                        │
  ┌────────▼────────┐     ┌─────────▼───────┐
  │  TANK   0x02   │◄───►│  POND    0x03   │
  │  (this device) │     │  remote pump    │
  └─────────────────┘     └─────────────────┘
```

| Node | Address | Role |
|------|:-------:|------|
| Gateway | `0x01` | Central coordinator |
| **Tank Controller** | **`0x02`** | **This device** |
| Pond Remote | `0x03` | Remote pump node |

---

## Hardware

```
┌──────────────────────────────────────────────────────────────┐
│                        ESP32 DevKit                          │
│                                                              │
│  SPI (LoRa E22)          Outputs          Inputs            │
│  ┌──────────────┐    ┌───────────────┐  ┌──────────────┐   │
│  │ NSS    →  5  │    │ Relay P1 → 13 │  │ BTN MODE→32  │   │
│  │ DIO1   → 26  │    │ Relay P2 → 15 │  │ BTN P1  →33  │   │
│  │ NRST   → 14  │    │ Relay P3 → 17 │  │ BTN P2  →25  │   │
│  │ BUSY   → 27  │    │ LED Strip→ 12 │  │ BTN P3  →21  │   │
│  │ SCK    → 18  │    └───────────────┘  │ BTN POND→22  │   │
│  │ MISO   → 19  │                       │ Float 0 →34  │   │
│  │ MOSI   → 23  │    ADC (Current)      │ Float 1 →35  │   │
│  └──────────────┘    ┌───────────────┐  │ Float 2 →36  │   │
│                      │ Curr P1 →  4  │  └──────────────┘   │
│                      │ Curr P2 → 39  │                      │
│                      │ Curr P3 →  2  │                      │
│                      └───────────────┘                      │
└──────────────────────────────────────────────────────────────┘
```

### Pin Summary

| GPIO | Function | Type | Notes |
|------|----------|------|-------|
| 2 | Current P3 ADC | ADC2 input | Must be < 0.6 V at power-on (pump off = safe) |
| 4 | Current P1 ADC | ADC2 input | |
| 5 | LoRa NSS | SPI CS | |
| 12 | WS2812B LED | Digital out | 10 LEDs |
| 13 | Relay P1 | Digital out | |
| 14 | LoRa NRST | Digital out | |
| 15 | Relay P2 | Digital out | |
| 17 | Relay P3 | Digital out | Safe output, no strapping concerns |
| 18 | LoRa SCK | SPI | |
| 19 | LoRa MISO | SPI | |
| 21 | Button P3 | Input pullup | |
| 22 | Button POND | Input pullup | |
| 23 | LoRa MOSI | SPI | |
| 25 | Button P2 | Input pullup | |
| 26 | LoRa DIO1 | IRQ input | |
| 27 | LoRa BUSY | Digital input | |
| 32 | Button MODE | Input pullup | |
| 33 | Button P1 | Input pullup | |
| 34 | Float switch 0 | Input only | Needs 10 kOhm pull-up to 3.3 V |
| 35 | Float switch 1 | Input only | Needs 10 kOhm pull-up to 3.3 V |
| 36 | Float switch 2 | Input only | Needs 10 kOhm pull-up to 3.3 V |
| 39 | Current P2 ADC | ADC1 input | Input only, safe |

**Total: 21 GPIO used** — Freed vs previous: GPIO 0 (strapping), GPIO 3 (UART-RX/non-ADC), GPIO 16 (DHT22 removed)

---

## LED Status Strip (10 x WS2812B)

```
LED:  [ 0 ][ 1 ][ 2 ][ 3 ][ 4 ][ 5 ][ 6 ][ 7 ][ 8 ][ 9 ]
       ─────────────── ──── ──── ──── ──── ──── ────
       Water Level     Stat Mode  P1   P2   P3  Pond
```

### Water Level (LEDs 0-2)

```
  Empty  o o o      Level 0 — triggers pond pump in auto mode
  Low    * o o      Level 1
  Mid    * * o      Level 2
  Full   * * *      Level 3 — stops pond pump in auto mode

  * = AQUA  o = off
```

### Status LED (LED 4) — priority order

```
  RED   solid    Fault lockout (overcurrent or dry-run)
  RED   flash    Comms error — gateway or pond timeout (400 ms)
  PURPLE         LoRa packet received (200 ms flash)
  CYAN           LoRa packet transmitted (200 ms flash)
  YELLOW pulse   Awaiting ACK from pond node
  GREEN          Auto mode — all OK
  BLUE           Manual mode — all OK
```

### Mode and Pump LEDs (LEDs 5-9)

| LED | GREEN / CYAN means | DIM means |
|-----|-------------------|----------|
| 5 | Auto mode (green) | Manual mode (blue) |
| 6 | Pump P1 running | P1 off (dim red) |
| 7 | Pump P2 running | P2 off (dim red) |
| 8 | Pump P3 running | P3 off (dim red) |
| 9 | Pond pump running (cyan) | Pond off (dim blue) |

---

## Features

```
  Auto mode       Float switches trigger pond pump automatically
  Manual mode     5 buttons control all pumps locally
  Pump hysteresis Min runtime + cooldown enforced per pump
  Overcurrent     Grace period filters inrush; warn or lockout (configurable)
  Dry-run         Same grace period; warn or lockout (configurable)
  ACK handshake   Pond commands confirmed with retry logic
  Net monitoring  Tracks gateway + pond contact; flags timeout
  LED feedback    10-LED strip shows full system state
  LoRa config     Gateway can read/write all settings at runtime
  NVS persist     Config, stats, mode survive reboots
```

---

## LoRa Packet Protocol v1.1

### Packet Layout (38 bytes total)

```
 Byte  0    1    2         3         4       5        6 ... 37
      +----+----+----------+---------+-------+--------+------------------+
      |  magic (0x5A6B)   |target_id|send_id| msg_id | msg_type| payload |
      +----+----+----------+---------+-------+--------+------------------+
       <---------- Header: 6 bytes ---------->  <-- Payload union: 32 B -->
```

### Message Types

```
  ID   Name              Direction          Description
  ---  ----------------  -----------------  ------------------------------------
   1   MSG_TELEMETRY     Node -> Gateway    Periodic sensor data
   2   MSG_COMMAND       Gateway -> Node    Pump on/off command
   3   MSG_ACK           Node <-> Node      Acknowledge receipt
   4   MSG_CONFIG_GET    Gateway -> Node    Request current config
   5   MSG_CONFIG_SET    Gateway -> Node    Write config to NVS
   6   MSG_CONFIG_RESP   Node -> Gateway    Config echo (verify applied)
   7   MSG_STATS_GET     Gateway -> Node    Request operational stats
   8   MSG_STATS_RESP    Node -> Gateway    Pump runtimes, faults, etc.
```

### Telemetry Payload (8 bytes)

```c
struct TelemetryData {
    uint8_t  water_level;   // 0-3  (3 float switches)
    uint8_t  system_flags;  // bit0=Auto bit1=P1 bit2=P2 bit3=P3 bit4=Pond
    int16_t  temperature;   // degrees C x10  (0 = DHT22 not fitted)
    uint8_t  humidity;      // % RH            (0 = DHT22 not fitted)
    uint8_t  error_code;    // 0=OK  1=Overcurrent  2=DryRun  3=Comms
    int8_t   last_rssi;     // dBm of last received packet
    int8_t   last_snr;      // SNR in dB
};
```

### RF Configuration

```
  Frequency   868.0 MHz  (EU ISM band)
  Bandwidth   125 kHz
  SF          9  (minimum; raise to SF10/SF11 for more margin at range)
  Coding rate 4/5
  Sync word   0x12
  TX power    22 dBm
  Preamble    8 symbols
  Airtime     ~330 ms for 38 bytes at SF9  (transmit() blocks the LoRa task)
  Round-trip  ~700 ms CMD + ACK  →  ACK timeout default 10 s gives ample margin
```

---

## Runtime Configuration

All parameters live in NVS, tunable over LoRa via `MSG_CONFIG_SET` without reflashing:

```
  Parameter                    Default      Range                Description
  ---------------------------  ----------   -------------------  --------------------------------
  pump_min_runtime_ms             30 000    5 000 - 3 600 000   Min ON time per pump
  pump_min_cooldown_ms            60 000    5 000 - 3 600 000   Min OFF before restart
  replenish_runon_ms             300 000   30 000 - 86 400 000  Min pond pump run (fill)
  telemetry_interval_ms           30 000    5 000 - 3 600 000   Telemetry send period
  network_timeout_ms              60 000   10 000 - 3 600 000   Contact timeout -> error
  ack_timeout_ms                  10 000    1 000 - 60 000       Wait for ACK before retry
  overcurrent_thresh               3 200        0 - 4 095        ADC raw cutoff (12-bit)
  dryrun_thresh                      150        0 - overcurrent  ADC raw floor
  ack_max_retries                      5        1 - 10           Retries before comms error
  boot_auto_mode                       1        0 or 1           Startup mode preference
  overcurrent_grace_ticks              5        0 - 255          Ticks (×50 ms) before fault trips
  fault_lockout_enabled                1        0 or 1           1=kill relays  0=warn only
```

---

## Persistent Statistics

Stored in NVS, survives reboots, queryable via `MSG_STATS_GET`:

```
  runtime_p1_s     Pump P1 total ON time (seconds)
  runtime_p2_s     Pump P2 total ON time (seconds)
  runtime_p3_s     Pump P3 total ON time (seconds)
  runtime_pond_s   Pond pump total ON time (seconds)
  fill_cycles      Tank refill count  <-- high count = possible leak
  boot_count       Total reboots
  last_fault       1 = overcurrent  /  2 = dry-run
```

---

## FreeRTOS Task Architecture

```
  Core 0                            Core 1
  +----------------------+          +----------------------+
  |  LoRaTransceiver     |          |  ControlEngine       |
  |  Priority 2          |          |  Priority 4  50 ms   |
  |  Event-driven        |          |  Relay/pump logic    |
  |  TX queue drain      |          |  OC grace counter    |
  |  RX ISR -> queue     |          |  ACK retry logic     |
  +----------+-----------+          |  Telemetry timer     |
             |  RX queue            +----------+-----------+
  +----------v-----------+                     |  mutex
  |  UIAnimation         |          +----------v-----------+
  |  Priority 1  30 ms   |          |  InputSensorPoll     |
  |  10-LED strip        |          |  Priority 3  20 ms   |
  |  Level/pump/net/mode |          |  Float switches      |
  +----------------------+          |  Buttons (debounce)  |
                                    |  ADC current (x8)    |
                                    +----------------------+

  Priority rationale:
    P4  ControlEngine   — relay decisions are most time-critical
    P3  InputSensorPoll — must deliver fresh sensor data promptly
    P2  LoRaTransceiver — comms important but not safety-critical
    P1  UIAnimation     — display only; freely preemptable

  Shared resources:
    StateMutex          guards SystemState struct
    LoRaIrqSemaphore    DIO1 ISR -> LoRaTransceiver task
    TxQueue (depth 8)   any task -> LoRaTransceiver
    RxQueue (depth 8)   LoRaTransceiver -> ControlEngine
```

---

## Operating Modes

### Auto Mode

```
  waterLevel == 0?
    YES -> canTurnPumpOn?
             YES -> Send CMD(pond ON) -> start replenish timer -> await ACK (retry x5, 10 s timeout)

  waterLevel >= 3 AND runOn expired?
    YES -> canTurnPumpOff?
             YES -> Send CMD(pond OFF) -> await ACK
```

### Manual Mode

```
  Button press -> toggle pump -> hysteresis check -> relay on/off
                                                  -> LoRa CMD if pond pump
```

### Fault / Overcurrent

```
  Inrush grace:
    ADC must exceed threshold for overcurrent_grace_ticks × 50 ms (default 250 ms)
    before any action is taken — short inrush spikes are ignored.

  fault_lockout_enabled = 1  (default):
    ADC > overcurrent_thresh  ->  Kill all relays  ->  errorCode = 1  ->  save NVS
    ADC < dryrun_thresh       ->  Kill all relays  ->  errorCode = 2  ->  save NVS
    To clear: press MODE button

  fault_lockout_enabled = 0  (warn-only):
    Same conditions  ->  errorCode = 1 or 2  (LED shows fault colour)
    Relays keep running; errorCode auto-clears when current returns to normal
    Useful during commissioning or for systems with high inrush motors

  Note: comms error (code 3) never triggers relay lockout in either mode
```

---

## Building & Flashing

### Dependencies (PlatformIO, auto-managed)

```ini
lib_deps =
    jgromes/RadioLib @ ^6.6.0
    FastLED/FastLED @ ^3.6.0
    thomasfredericks/Bounce2 @ ^2.71.0
```

### Commands

```bash
# Build
platformio run -e esp32dev

# Flash
platformio run -e esp32dev --target upload

# Monitor (115200 baud)
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
+-- platformio.ini
+-- src/
    +-- main.cpp          (~51 KB, 17 sections)
        +-- S1   Pin definitions
        +-- S2   Compile-time constants & RF settings
        +-- S3   Packed network protocol structs
        +-- S4   Global system state
        +-- S5   FreeRTOS handles
        +-- S6   Peripheral instances
        +-- S7   Replay-attack sequence tracking
        +-- S8   RadioLib DIO1 ISR
        +-- S9   NVS load / save
        +-- S10  Shared helpers (telemetry, ACK, commands)
        +-- S11  Pump hysteresis management
        +-- S12  Task: InputSensorPoll  (Core 1, P3, 20 ms)
        +-- S13  Task: ControlEngine    (Core 1, P4, 50 ms)
        +-- S14  Task: LoRaTransceiver  (Core 0, P2, event)
        +-- S15  Task: UIAnimation      (Core 0, P1, 30 ms)
        +-- S16  setup()
        +-- S17  loop()  -- intentionally empty
```

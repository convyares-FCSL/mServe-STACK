# mserve_interfaces

Shared ROS 2 interfaces for the mServe and HyFleet stack.
All messages, services, and actions used across packages are defined here.

---

## Interface boundary principle

The PLC uses a fixed hardware command contract (TwinCAT function block):

```
id       : UDINT   — target device/booster identifier
cmd      : UDINT   — encoded command enum (E_Cmd)
payload  : ARRAY [1..5] OF UDINT — command-specific data
ackID    : UDINT   — echoed back on acknowledgement
result   : HRESULT — signed 32-bit result code
```

**This contract is never exposed inside ROS.** The ADS bridge node is the only
place that knows about `id/cmd/payload` encoding. Inside ROS, all interfaces use
named, typed fields. The bridge translates in both directions.

```
ROS node  →  clean typed service  →  ADS bridge  →  id/cmd/payload  →  PLC
ROS node  ←  bool success/message ←  ADS bridge  ←  ackID/result    ←  PLC
```

---

## HyFleet compression interfaces

### Actions

#### `action/ControlCompressor.action`
External entry point. Sent by the orchestrator to `hyfleet_compression`.

```
Goal:
  uint8 START = 1
  uint8 STOP = 2
  uint8 SAFE_STOP = 3

  uint8 LOW_BOOSTER = 1
  uint8 HIGH_BOOSTER = 2
  uint8 SYNC_BOOSTERS = 3

  uint8 command
  uint8 target
  float64 target_pressure

Result:
  bool accepted
  string message

Feedback:
  float64 pressure
  float64 percent_complete
```

#### `action/ControlBooster.action`
Sent by `hyfleet_compression` coordinator to each `hyfleet_booster` instance.
`START_IDLE` boosts to target then holds (poppet valve engaged, VFD available
for rapid restart). Used during SYNC to avoid cold-starting the low booster.

```
Goal:
  uint8 START = 1
  uint8 START_IDLE = 2
  uint8 STOP = 3
  uint8 SAFE_STOP = 4

  uint8 command
  float64 target_pressure

Result:
  bool accepted
  string message

Feedback:
  float64 pressure
  float64 percent_complete
```

---

### Booster hardware services

#### `srv/BoosterCmd.srv`
Single multiplexed service for all booster hardware commands. One instance per
booster, namespaced: `/low_booster/booster_cmd`, `/high_booster/booster_cmd`.
The ADS bridge advertises these and translates named fields to the PLC wire format.

```
uint8 START_VFD  = 1    # Start VFD at speed_rpm
uint8 STOP_VFD   = 2    # Stop VFD
uint8 SET_PCSV   = 3    # Enable/disable PCSV at cpm rate
uint8 CONTROL_SV = 4    # Open/close solenoid valve by device_id

uint8   cmd
bool    enable       # SET_PCSV / CONTROL_SV — energise the targeted device
string  device_id    # CONTROL_SV — which valve; bridge owns the lookup
float64 speed_rpm    # START_VFD  — VFD speed (rpm)
float64 cpm          # SET_PCSV   — compression rate (cpm)
---
bool   success
string message
```

ADS bridge payload encoding (internal — not visible to ROS callers):

| cmd | payload[0] | payload[1] |
|---|---|---|
| START_VFD | speed_rpm (rounded) | — |
| STOP_VFD | 0 | — |
| SET_PCSV | enable (0/1) | cpm (rounded) |
| CONTROL_SV | state (0/1) | — |

#### `srv/Cmd.srv`
Raw PLC command contract — mirrors the TwinCAT `ST_Cmd` function block.
Use only when bypassing typed subsystem services is explicitly required.
Normal callers use `BoosterCmd` or equivalent typed services.

```
uint32   id
uint32   cmd
int32[5] payload
---
uint32 ack_id
int32  result       # HRESULT
```

#### `srv/CompressorCmd.srv`
Heater control for the compressor subsystem.
```
uint8 CONTROL_HEATER = 1

uint8 cmd
bool    enable      # CONTROL_HEATER — on/off
float64 setpoint    # CONTROL_HEATER — temperature setpoint
---
bool   success
string message
```

#### `srv/DispenserCmd.srv`
Dispenser control — start/stop dispensing, user session management.
```
uint8 START = 1
uint8 STOP = 2
uint8 LOGIN = 3
uint8 LOGOUT = 4
uint8 PAYMENT = 5

uint8   cmd
bool    enable
string  guid        # user/session identifier
float64 pressure    # target dispensing pressure
---
bool   success
string message
```

#### `srv/GasRouterCmd.srv`
Open or close a named gas routing path.
```
uint8 OPEN_PATH  = 1
uint8 CLOSE_PATH = 2

uint8  cmd
string path_id    # bridge owns the valve mapping for this path
---
bool   success
string message
```

---

### Telemetry

#### `msg/CompressorTelemetry.msg`
Published by the ADS bridge at ~10 Hz. Single topic for the entire compressor
subsystem — all pressure transducers, temperatures, and VFD data in one message.
Both `BoosterNode` instances subscribe and extract their slice using config-driven
array indices. Shared sensors (e.g. interstage PT) are accessible to both.

```
Topic:  compressor_telemetry
Rate:   ~10 Hz

Key fields:
  float64[10] hbu_pt_bar       — pressure transducers (bar)
  float64[12] hbu_tt_celsius   — temperature transducers
  float64     vfd1_speed_rpm   — low booster VFD speed
  float64     vfd2_speed_rpm   — high booster VFD speed

Booster index mapping (from config params):
  low_booster:  inlet=hbu_pt_bar[0], outlet=hbu_pt_bar[7], vfd=vfd1_speed_rpm
  high_booster: inlet=hbu_pt_bar[1], outlet=hbu_pt_bar[2], vfd=vfd2_speed_rpm
```

---

## Mobile base interfaces

- `msg/DriveStatus`, `msg/WheelCommand`, `msg/WheelFeedback`
- `msg/DisplayStatus`, `msg/Esp32Status`
- `srv/SetDisplayMode`
- `action/Dock`

---

## Build

```bash
colcon build --packages-select mserve_interfaces --allow-overriding mserve_interfaces
source install/setup.bash
ros2 interface show mserve_interfaces/srv/StartVFD
```

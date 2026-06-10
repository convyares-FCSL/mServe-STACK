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
  uint8 START      = 1
  uint8 STOP       = 2
  uint8 FORCE_STOP = 3

  uint8 LOW_BOOSTER   = 1
  uint8 HIGH_BOOSTER  = 2
  uint8 SYNC_BOOSTERS = 3

  uint8   command
  uint8   target
  float64 target_pressure

Result:
  bool   accepted
  string message

Feedback:
  float64 pressure
  float64 percent_complete
```

Mode (PERFORMANCE / ECO) is not a goal field — it is persistent coordinator state
set via the `~/set_mode` service before a fill begins.

#### `action/ControlBooster.action`
Sent by `hyfleet_compression` coordinator to each `hyfleet_booster` instance.

`ON_TARGET_HOLD` compresses to target then holds in a maintain band — PCSV off when
at pressure, re-engage when outlet drops below `reenable_pressure_bar`. VFD stays
running throughout. Used for interstage band-control in SYNC mode.

`INLET_STARVE_PAUSE` blocks PCSV (returns RUNNING) while inlet pressure is below
`inlet_starve_bar`, resuming when it recovers above `inlet_resume_bar`. Used by the
high booster in SYNC to pause while the low booster recovers interstage pressure.

```
Goal:
  uint8 COMPRESS   = 1
  uint8 STOP       = 2
  uint8 FORCE_STOP = 3

  uint8 ON_TARGET_SUCCEED = 0   ← return SUCCESS when target reached (one-shot)
  uint8 ON_TARGET_HOLD    = 1   ← maintain band; loop until preempted then STOP

  uint8 INLET_STARVE_ABORT = 0  ← abort goal if inlet drops below starve threshold
  uint8 INLET_STARVE_PAUSE = 1  ← pause PCSV while starved; resume on recovery

  uint8   command
  float64 target_pressure       ← validated against per-instance min/max on goal-accept
  float64 cpm          16.0     ← coordinator-decided; validated against CPM_MAX constexpr
  float64 speed_rpm    1500.0   ← coordinator-decided; validated against SPEED_MAX_RPM
  uint8   on_target       0     ← 0 = succeed once, 1 = hold band
  uint8   on_inlet_starve 0     ← 0 = abort, 1 = pause
  float64 inlet_starve_bar -1.0 ← starvation threshold (-1 = not used)
  float64 inlet_resume_bar -1.0 ← recovery threshold (-1 = not used)

Result:
  bool   accepted
  string message

Feedback:
  float64 pressure
  float64 percent_complete
```

---

### Compression control services

#### `srv/BoosterCmd.srv`
Single multiplexed service for all booster hardware commands. One instance per
booster, namespaced: `/low_booster/booster_cmd`, `/high_booster/booster_cmd`.
The ADS bridge advertises these and translates named fields to the PLC wire format.

```
uint8 START_VFD  = 1    # Start VFD at setpoint (rpm)
uint8 STOP_VFD   = 2    # Stop VFD
uint8 SET_PCSV   = 3    # Enable/disable PCSV; setpoint = cpm
uint8 CONTROL_SV = 4    # Open/close solenoid valve at index

uint8   cmd
bool    enable      # SET_PCSV / CONTROL_SV — energise the targeted device
uint8   index       # CONTROL_SV — index into CompressorTelemetry::sv[5]
float64 setpoint    # START_VFD: speed (rpm) / SET_PCSV: compression rate (cpm)
---
bool   success
string message
```

#### `srv/CompressorCmd.srv`
Coordinator-level service for system-wide actuation: interstage solenoid valve
and oil heater. Advertised at `/hyfleet_compression/compressor_cmd`.

```
uint8 INVALID        = 0
uint8 CONTROL_SV     = 1    # Open/close a solenoid valve by index
uint8 CONTROL_HEATER = 2    # Enable/disable oil heater; setpoint = target °C

uint8   cmd
bool    enable      # CONTROL_SV / CONTROL_HEATER — energise the device
uint8   index       # CONTROL_SV — index into CompressorTelemetry::sv[5]
float64 setpoint    # CONTROL_HEATER — temperature setpoint (°C)
---
bool   success
string message
```

#### `srv/SetMode.srv`
Sets the compression mode on `hyfleet_compression`. Mode persists across fills —
set once before a session begins. Replaces the per-goal `mode` field that was
previously on `ControlCompressor.action`.

```
uint8 PERFORMANCE = 1
uint8 ECO         = 2

uint8 mode
---
bool   success
string message
```

#### `srv/DispenserCmd.srv`
Dispenser control — start/stop dispensing, user session management.
```
uint8 START   = 1
uint8 STOP    = 2
uint8 LOGIN   = 3
uint8 LOGOUT  = 4
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
subsystem — all pressure transducers, temperatures, VFD data, and valve states
in one message. Both `BoosterNode` instances and `CompressorNode` subscribe and
extract their slice using config-driven array indices.

```
Topic:  compressor_telemetry
Rate:   ~10 Hz

uint8 mode                  # PLC operating state: OFF/STARTUP/AUTO/MANUAL/LOCKOUT

float64[16] pt_bar          # all pressure transducers (bar)
float64[12] tt_celsius      # all temperature transducers (°C)
bool[5]     sv              # solenoid valve states
bool[4]     ps              # end-of-travel position switches

uint8[2]   vfd_state        # VFD_OFFLINE=0 VFD_IDLE=1 VFD_RUNNING=2 VFD_FAULT=3
float64[2] vfd_speed_rpm    # VFD speed (rpm)
float64[2] vfd_energy_kj    # VFD energy (kJ)
float64[2] vfd_power_kw     # VFD power (kW)

bool    heater_on           # PLC handles both physical heaters in parallel
float64 hpu_tt_celsius      # Hydraulic tank temperature (°C)
float64 hpu_ls_percent      # Hydraulic tank fill level (%)

builtin_interfaces/Time timestamp
```

Array index mapping (from per-instance config params, not hardcoded):

| Signal | `low_booster` | `high_booster` | `hyfleet_compression` |
|---|---|---|---|
| Inlet PT | `pt_bar[0]` | `pt_bar[1]` | `pt_bar[0]` |
| Outlet / interstage PT | `pt_bar[7]` | `pt_bar[2]` | `pt_bar[7]` |
| VFD | `vfd_*[0]` | `vfd_*[1]` | — |
| Interstage SV | — | — | `sv[4]` |

---

## Mobile base interfaces

- `msg/DriveStatus`, `msg/WheelCommand`, `msg/WheelFeedback`
- `msg/DisplayStatus`, `msg/Esp32Status`
- `srv/SetDisplayMode`
- `action/Dock`

---

## Build

```bash
colcon build --packages-select mserve_interfaces
source install/setup.bash

# Inspect any interface
ros2 interface show mserve_interfaces/action/ControlBooster
ros2 interface show mserve_interfaces/srv/SetMode
```

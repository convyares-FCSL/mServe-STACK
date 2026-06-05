# HyFleet Compression Module — System Description

## Physical Hardware

**Haskel HBU (Booster Unit)** + **Boosh HPU (Hydraulic Power Unit)** form one compression station.

The HPU contains two boosters:
- **Low booster** — 35 to 300 bar range
- **High booster** — 200 to 900 bar range

Boosters can run **independently** (parallel — each fills from the supply) or **in series**
(low booster feeds into high booster, with a sync solenoid valve between stages to select mode).

---

## Booster Internals

Each booster has the same structure, just different operating parameters:

| Device | ID pattern | Purpose |
|--------|-----------|---------|
| Inlet SV | `hbu_inlet_1` / `hbu_outlet_2` | Opens gas path into booster |
| HPU SV | `hpu_sv_1` / `hpu_sv_2` | Energises hydraulic drive |
| VFD | `booster1_vfd` / `booster2_vfd` | Motor speed control (0–6000 RPM) |
| PCSV | `booster1_pcsv` / `booster2_pcsv` | Compression cycle valve (0–18 CPM) |

The **PCSV CPM** (cycles per minute) is the primary compression rate control.
The **VFD** sets the hydraulic pump speed — ramp up before PCSV starts, ramp down after stop.

---

## Common / Shared Systems

- **Oil temperature** — monitored and used for safety inhibit; heaters controlled to maintain range
- **Oil level** — status indicator only (no control)
- **Sync solenoid** — selects series vs parallel mode between the two boosters

---

## Booster Startup Sequence (State Machine)

The archive implements this as a hand-coded enum state machine in `Booster::update()`:

```
WAIT_COMMAND
  → start received: open inlet SV
RAMP_VFD_UP
  → wait for inlet pressure stable (rolling window, 3 samples within tolerance)
VFD_DELAY
  → start VFD at target speed, wait N control cycles
WAIT_RAMP
  → wait for VFD speed >= target − tolerance
WAIT_STABILIZATION
  → wait N control cycles for system to settle
ENERGISE_VALVE
  → open HPU solenoid
PCSV_ON
  → run PCSV at target CPM; monitor:
    - inlet pressure drop below 0.2 bar → stop PCSV
    - outlet pressure >= target → success, stop PCSV
PCSV_OFF / DEENERGISE_VALVE / RAMP_VFD_DOWN / WAIT_STOP
  → ordered shutdown: stop PCSV → close HPU SV → ramp VFD down → wait VFD <= 25 RPM → close inlet SV
```

---

## Stop Modes

| Mode | Behaviour |
|------|-----------|
| **Stop** | Clear start flag; booster walks through the full ordered shutdown sequence |
| **Safe stop** | Set safe_stop flag; exits current active state and enters ordered shutdown sequence |
| **Force stop** | Immediate: turn off all solenoids, shut down drives — no wait, no checks. Rare/fault event |

The archive only implements stop and safe_stop cleanly. Force stop is described but not fully
implemented — it would need to bypass the state machine and issue immediate device commands.

---

## Safety Monitoring

Performed on every telemetry update. Any violation **inhibits** both boosters:

| Check | Limit | Action on fault |
|-------|-------|----------------|
| Oil temp high | > 60 °C | Inhibit, turn heaters OFF |
| Oil temp low | < 15 °C | Inhibit, turn heaters ON |
| HBU inlet temperature | > 50 °C | Inhibit, turn heaters OFF |
| HBU outlet temperature | > 190 °C | Inhibit, turn heaters OFF |
| Chiller outlet temperature | > 70 °C | Inhibit, turn heaters OFF |
| Telemetry timeout | > 2 s | Inhibit |

Heater IDs: `[1, 2]` — controlled via `DeviceCommands.heater` dispatch.

---

## ROS Interfaces

### Incoming (subscribes / serves)
| Interface | Type | Topic/Name | Purpose |
|-----------|------|-----------|---------|
| Telemetry subscriber | Topic sub | `compressor_telemetry` | Live sensor data from PLC |
| Control action server | Action | `control_compressor` | External commands (start/stop/safe_stop) |

### Outgoing (publishes / calls)
| Interface | Type | Topic/Name | Purpose |
|-----------|------|-----------|---------|
| Diagnostics publisher | Topic pub | `compressor_diagnostics` | Status to operator console |
| Low booster command | Service client | `control_booster_1` | Sends device commands to PLC |
| High booster command | Service client | `control_booster_2` | Sends device commands to PLC |
| Compressor command | Service client | `control_compressor` | Sends compressor-level commands to PLC |

### Action interface (`ControlCompressor`)
```
Goal:
  uint8 command    # START=1, STOP=2, SAFE_STOP=3
  uint8 target     # LOW_BOOSTER=1, HIGH_BOOSTER=2, SYNC_BOOSTERS=3
  float64 target_pressure  # bar (validated against per-target limits)

Feedback:
  float64 pressure          # current output pressure
  float64 percent_complete  # 0–100

Result:
  bool accepted
  string message
```

Pressure limits: low booster 0–350 bar, high booster 0–950 bar.

---

## Telemetry Message (`CompressorTelemetry`)

- 12 HBU temperature readings — mapped by index to inlet/outlet per booster
- HPU oil temperature
- Per-booster: input/output pressure, input/output temperature, compression ratio, VFD energy, VFD speed
- Index mappings (from config):
  - Low booster: inlet PT index 0, outlet PT index 7, inlet TT 0, outlet TT 2, VFD index 1
  - High booster: inlet PT index 1, outlet PT index 2, inlet TT 4, outlet TT 6, VFD index 2
  - Safety inlet temps: indices 0, 1, 4, 5
  - Safety outlet temps: indices 2, 3, 6, 7
  - Chiller temps: indices 8, 9

---

## Problems With Archive Code (Why Rebuild)

1. **Monolithic lifecycle node** — `CompressorNode` directly manages action server, telemetry,
   safety monitor, device dispatch, and the booster manager. Hard to test in isolation.

2. **Hand-coded state machine** — `Booster::update()` is a large switch/case run on a timer tick.
   No visibility, hard to introspect, state transitions are implicit flags.

3. **Stop/force-stop logic scattered** — safe_stop is a flag in the booster; force stop not
   implemented; no clean separation between "operator request" and "safety fault" paths.

4. **No reactive safety** — safety evaluation happens in the timer callback, not as a first-class
   interrupt. If safety fails mid-cycle, it depends on the booster state machine checking flags.

5. **Mutex-heavy action server** — `CompressorAction` manually manages thread safety with a mutex
   across goal queuing, status updates, and cancellation. Error-prone.

6. **No visualisation / debuggability** — no way to watch state transitions live.

---

## New Package: `hyfleet_compression`

Located at: `ws/src/hyfleet_compression/`

**Target architecture:** BehaviorTree.ROS2 replaces the hand-coded state machine. Key changes:

- Each booster startup sequence → `RosActionNode` (`RunBoosterCycle`) wrapping the actual
  hardware start/run/stop as a long-running action goal
- Safety conditions → `RosTopicSubNode` conditions checked every tick:
  `OilTempOK`, `OilLevelOK`, `HbuTempOK`, `TelemetryFresh`
- Reactive monitoring → `Parallel` node with safety monitor running alongside the compression cycle
- Force stop → preemption path: dedicated subtree that issues immediate device commands, bypasses
  the normal shutdown sequence
- Lockout / final state → BT latched-failure state requiring explicit reset service before the
  tree can re-enter RUNNING
- Full system → wrapped in `TreeExecutionServer` so HyQube orchestrator can trigger it
  as one step in the fill sequence

The booster device logic (PCSV, VFD, SV commands) is kept as C++ helpers; the BT drives
*when* to call them, not the *how*.

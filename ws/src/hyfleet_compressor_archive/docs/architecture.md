# Compressor Module — Architecture

All decisions in this document are **CLOSED**. Do not reopen without new information.
Use the principles below to resolve future questions consistently.

---

## Principles

- **Signals, not tentacles.** Cross-node interaction is ROS actions/services/topics only. Never one node calling another's methods, never a shared blackboard. Necessary physical coupling (SYNC interstage, shared oil) is expressed as observable interfaces; accidental coupling is removed.
- **Fact vs instruction.** A *fact about a node* (index, limits, timing) is a parameter; a *value that's part of an instruction* (a setpoint, a mode) travels with the goal/command.
- **Per-instance vs universal fact.** A fact that *differs per instance* → config parameter. A fact *universal to the hardware type and immutable* → `constexpr`. A constant cannot be set at all — stronger than reject-while-active.
- **Command, not param, for actuator setpoints.** Anything that moves hardware arrives through a validated, acknowledged, traceable command — never a fire-and-forget `param set`.
- **Fail-safe on stale.** A safety/permission signal that is absent or stale reads as *unsafe*, not "last known good".
- **Share a field only when it's one honest abstraction.** Merge when the meaning is identical (on/off); split when merging would hide a unit difference.
- **Real-time by cycle time.** Loops faster than ~100 ms (hard real-time — VFD ramp at 5 ms, fast interlocks) execute in the PLC. Loops at ~100 ms and slower live in ROS 2, including genuine closed-loop control (pressure, CPM trim, heating ~1 Hz, dispensing ~2 Hz). ROS 2 closes the slow loops; it never owns a sub-100 ms one.

---

## Control authority — three layers

| Layer | Protects | Owner | Examples | Recovery |
|---|---|---|---|---|
| **Process control** | operating envelope — reaching and holding spec | **ROS 2** | heater regulation (~1 Hz), operating-range gate (e.g. oil 0–50 °C), pressure and CPM control | auto — clears when back in range |
| **Machine control** | equipment | **PLC** | device alarm/fault → sub-system lockout (e.g. oil < -10 °C); all sub-100 ms loops including VFD ramp | latched — requires clearing |
| **Health control** | humans | **Safety PLC** (separate device) | personnel and functional safety | per the safety system |

Rules:
- Pick the authority by what's at stake: harmless-but-invalid operating point → ROS; equipment damage → PLC; human harm → Safety PLC.
- Cycle-time override: process control needing sub-100 ms determinism executes in the PLC — ROS commands the setpoint, PLC runs the loop.
- **Protective layers act first, inform ROS after.** Equipment/personnel safety never depends on ROS being up, responsive, or correct. ROS reacts to *declared* state (warn/alarm/lockout flags in telemetry) — it does not re-derive danger from raw sensor values.
- **Recoverability differs:** ROS operational gate auto-clears when back in range; PLC device lockout is latched and needs clearing; Safety PLC trip sits above both.
- ROS force-stop and PLC autonomous trip are two **independent** stop sources — kept distinct. The plant stays protected via PLC/Safety PLC even if the ROS force-stop never fires.

---

## Node layout

Nav2-style pattern. Both coordinator and capability nodes are **lifecycle nodes** that own their own action servers and tick BT trees internally. No direct class coupling between nodes. Blackboard is local to each node's tree — cross-node communication uses ROS interfaces only.

```
hyfleet_compression (CompressorNode)   ← lifecycle, rclcpp_action server, coordinator BT tree
                                         single external entry point for orchestrator
                                         bringup last — after boosters are active

hyfleet_booster × 2 (BoosterNode)      ← lifecycle, rclcpp_action server, booster BT state machine
                                         same binary: launched as low_booster + high_booster
                                         config-driven array indices distinguish instances
                                         bringup before coordinator
```

Bringup order: `base → drivechain → low_booster → high_booster → hyfleet_compression`

---

## Node pattern — identical for coordinator and boosters

```
LifecycleNode
  constructor   → declare_params() — all params declared with descriptor ranges; no read_only
  on_configure  → load_params()    — read params; store per-instance limits as members; write to blackboard
                  register BT nodes, load trees from XML
                  create rclcpp_action server (Reentrant callback group)
  on_activate   → set accepting_goals = true
  on_deactivate → set accepting_goals = false, abort active goal via action protocol
  on_cleanup    → destroy action server, destroy tree, reset factory

  on_parameters → if state == ACTIVE: reject all; else accept (effective on next on_configure)

  goal arrives  → validate goal fields against limits
                  write goal fields to blackboard (command, target_pressure, cpm, speed_rpm)
                  select tree, start tick timer
  tick timer    → tree.tickOnce()
  tree SUCCESS  → succeed the goal, stop timer
  tree FAILURE  → abort the goal, stop timer
```

---

## Action interfaces

### `ControlCompressor` (orchestrator → coordinator)

```
Goal:
  uint8 START = 1
  uint8 STOP = 2
  uint8 SAFE_STOP = 3

  uint8 LOW_BOOSTER = 1
  uint8 HIGH_BOOSTER = 2
  uint8 SYNC_BOOSTERS = 3

  uint8 PERFORMANCE = 1
  uint8 ECO = 2

  uint8 command
  uint8 target
  uint8 mode               ← performance / eco — coordinator owns mode → setpoint translation
  float64 target_pressure

Result:
  bool accepted
  string message

Feedback:
  float64 pressure
  float64 percent_complete
```

### `ControlBooster` (coordinator → booster)

```
Goal:
  uint8 START = 1
  uint8 START_IDLE = 2
  uint8 STOP = 3
  uint8 SAFE_STOP = 4

  uint8 command
  float64 target_pressure
  float64 cpm             ← coordinator-decided; live trims via adjust-CPM service
  float64 speed_rpm       ← coordinator-decided; default 1500 rpm (coordinator profile param)

Result:
  bool accepted
  string message

Feedback:
  float64 pressure
  float64 percent_complete
```

`START_IDLE`: boost to target then hold with poppet valve engaged, VFD ready for rapid restart.
Used in SYNC mode to avoid cold-starting the low booster.

`target_pressure` is goal-only, never a param. Validated against per-instance limits on goal-accept.
`cpm` and `speed_rpm` are coordinator-decided setpoints, validated against `constexpr` hardware
ceilings on goal-accept. The PLC ramps the VFD (5 ms) — ROS commands the target, PLC runs the loop.

---

## Hardware limits — `constexpr` vs param

Limits confirmed as fixed hardware manufacturer ceilings, identical on both boosters:

```cpp
// include/hyfleet_booster/booster_limits.hpp
constexpr double SPEED_MIN_RPM = 0.0;
constexpr double SPEED_MAX_RPM = 1600.0;
constexpr double CPM_MIN       = 0.0;
constexpr double CPM_MAX       = 18.0;
```

**Not parameters.** A `constexpr` cannot be set at all — stronger than reject-while-active. Removing the param eliminates the risk of an unsafe override of a guaranteed hardware ceiling. Log these at `on_configure` for commissioning visibility.

Per-instance limits (pressure varies low vs high booster) remain **parameters**, validated at goal-accept and stored as members after `load_params()`.

---

## Booster parameters

Two values are derived, not params:
- **Service name**: `/{node_name}/booster_cmd` — architecture contract
- **Telemetry topic**: `compressor_telemetry` — architecture contract. `BoosterNode` owns the
  subscription; BT nodes never subscribe directly. A thread-safe `BoosterTelemetryCache`
  (`shared_ptr` on blackboard) is the single source of truth for all telemetry consumers.

All params follow one rule: **not changeable while ACTIVE**. `on_parameters` rejects any change in that state. Changes made while INACTIVE or UNCONFIGURED take effect on the next `on_configure`.

### Hardware mapping (per-instance, set in launch file)

Array indices into `CompressorTelemetry` fields — differ between `low_booster` and `high_booster`.

`vfd_index`, `inlet_pt_index`, `outlet_pt_index`, `hyd_primer_pt_index`, `hyd_a_pt_index`,
`hyd_b_pt_index`, `coolant_pt_index`, `inlet_tt_index_1/2`, `outlet_tt_index_1/2/3`,
`ps_lhs_index`, `ps_rhs_index`, `inlet_sv_index`, `hpu_sv_index`

No `heater_index` — `heater_on` is a single bool; PLC handles both physical heaters in parallel.

### Operational (commissioning-tunable)

Tolerances, timing, safety thresholds, pressure limits. Tuned during commissioning.
`target_speed` and `target_cpm` are NOT in this list — they arrive with the goal.

`ramp_tolerance`, `stop_threshold`, `min_pressure_bar`, `max_pressure_bar`, `safe_pressure`,
`target_deadband`, `min_temp_inlet`, `max_temp_inlet`, `min_temp_outlet`, `max_temp_outlet`,
`stability_tolerance`, `vfd_delay_ms`, `vfd_stabilization_ms`, `stabilization_samples`,
`stability_timeout_ms`, `ramp_timeout_ms`, `stop_timeout_ms`

---

## Telemetry — `CompressorTelemetry`

Single topic `compressor_telemetry`, published by ADS bridge at ~10 Hz. `BoosterNode` owns one
subscription per instance; the callback writes to `BoosterTelemetryCache`. BT nodes and the tick
loop read via `cache->latest()` — no BT node holds a subscription. Array indices (PT, TT, VFD, SV,
PS) are config-driven params — same node binary serves both booster instances.

Key fields (current):
```
uint8 mode                      # PLC operating mode: OFF/STARTUP/AUTO/MANUAL/LOCKOUT
float64[16] pt_bar              # all pressure transducers
float64[12] tt_celsius          # all temperature transducers
bool[5]     sv                  # solenoid valve states
bool[4]     ps                  # end-of-travel position switches
uint8[2]    vfd_state           # VFD_OFFLINE=0 VFD_IDLE=1 VFD_RUNNING=2 VFD_FAULT=3
float64[2]  vfd_speed_rpm
float64[2]  vfd_energy_kj
float64[2]  vfd_power_kw
bool        heater_on           # single — PLC handles both physical heaters in parallel
float64     hpu_tt_celsius
float64     hpu_ls_percent
```

The `mode` field carries the PLC's declared operating state. ROS conditions key off `mode`
and per-device warn/alarm/lockout flags — not raw sensor values. This ensures ROS and the PLC
can never disagree about whether something is dangerous.

---

## Oil / temperature — operating-range gate

- **ROS (process):** runs heater regulation (~1 Hz) and the operating-range gate. Won't start if oil is outside operating band (e.g. < 0 or > 50 °C). An invalid operating point, not damage/harm. Published as `oil_healthy` topic. Auto-recovering when temperature returns to range.
- **PLC (machine):** equipment-protection threshold (e.g. oil < -10 °C) → device alarm/fault → sub-system lockout. Latched — requires clearing. BT reflects PLC lockout flag by moving to fault/lockout and refusing goals.
- Boosters subscribe to `oil_healthy` (`RosTopicSubNode`). **Fail-safe on stale:** if no fresh message within N seconds → treat as unhealthy. Stops ROS commanding blind; PLC/Safety PLC protect the plant regardless.
- Keep a trivial fake `oil_healthy` publisher for standalone booster testing.

---

## Booster BT — blackboard contents

Written at configure time (from params / architecture contracts):
`service_name`, `telemetry_cache` (shared_ptr — not a param, created at configure),
`vfd_index`, `inlet_pt_index`, `outlet_pt_index`, all other hardware indices,
`ramp_tolerance`, `stop_threshold`, `safe_pressure`, `target_deadband`,
`vfd_delay_ms`, `vfd_stabilization_ms`, `stabilization_samples`,
`stability_timeout_ms`, `ramp_timeout_ms`, `stop_timeout_ms`

Written at goal-accept time (from goal):
`target_pressure`, `cpm`, `speed_rpm`

---

## Coordinator BT — goal routing

Goal fields written to blackboard → tree routes on `target`:

```
target = LOW   → RosActionNode → /low_booster/control_booster
target = HIGH  → RosActionNode → /high_booster/control_booster
target = SYNC  → Parallel
                   ├── RosActionNode → /low_booster/control_booster   (START_IDLE)
                   └── RosActionNode → /high_booster/control_booster  (START)
```

The coordinator owns mode → setpoint translation. Profiles (CPM and speed per mode) are
coordinator parameters — commissioning-tunable, reject-while-active. Live CPM trim in SYNC
comes from interstage pressure (ROS-rate control loop running in the coordinator tree).

---

## BoosterCmd service — `srv/BoosterCmd.srv`

Single multiplexed service per booster instance. ADS bridge owns PLC wire-format encoding.
ROS never sees `id/cmd/payload`. Service namespaced per booster (`/low_booster/booster_cmd`).

```
uint8 START_VFD  = 1
uint8 STOP_VFD   = 2
uint8 SET_PCSV   = 3
uint8 CONTROL_SV = 4

uint8   cmd
float64 speed_rpm
float64 cpm
bool    enable
uint8   sv_index
---
bool   success
string message
```

`START_VFD` and `STOP_VFD` are kept discrete — stop must be unambiguous.

---

## Build stages

### Stage 1 — BoosterNode with action server + BT ✓
Lifecycle node, action server, BT factory, tree from XML. Simple placeholder tree.

### Stage 2 — Booster BT state machine ✓
Real state machine: RosServiceNode hardware commands, telemetry cache shared via blackboard,
StatefulActionNode wait nodes (VFDAtSpeed, VFDStopped, InletPressureStable, OutletAtPressure),
ConditionNode gate (InletPressureSafe), Parallel safety monitor, action feedback per tick.

### Stage 3 — CompressorNode coordinator
Action server + coordinator BT tree. RosActionNode calling booster servers.
Goal routing LOW / HIGH / SYNC Parallel.

### Stage 4 — Full integration
SYNC CPM control, interstage solenoid, safety monitor, diagnostics.

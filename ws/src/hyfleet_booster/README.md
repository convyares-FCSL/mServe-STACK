# hyfleet_booster

ROS 2 lifecycle node for a single hydrogen compression booster stage.
The same binary is launched twice — as `low_booster` and `high_booster` — with
different parameters driving all differences between instances.

Each instance owns an `rclcpp_action` server and an internal BehaviorTree.CPP tree.
The tree is the state machine. No hand-rolled switch statements.

---

## What it does

- Accepts `ControlBooster` action goals: `COMPRESS`, `STOP`, `FORCE_STOP`
- `on_target` field selects compress-once (`ON_TARGET_SUCCEED`) or maintain-band (`ON_TARGET_HOLD`)
- `on_inlet_starve` field selects abort (`INLET_STARVE_ABORT`) or pause (`INLET_STARVE_PAUSE`) on inlet loss
- Loads four XML trees at configure time — one per command
- Ticks the active tree on a 100 ms wall timer while a goal is active
- Publishes outlet pressure + percent_complete feedback each tick
- Succeeds or aborts the goal when the tree resolves
- Lifecycle managed: configure / activate / deactivate / cleanup / shutdown

---

## Drivetrain state model

Three hardware states drive the command semantics. OFF↔WARM is expensive;
WARM↔COMPRESSING is cheap. `ON_TARGET_HOLD` exists to pay the expensive transition
once and then cycle between WARM and COMPRESSING cheaply.

| State | VFD | HPU | PCSV | Notes |
|---|---|---|---|---|
| **OFF** | Stopped | Closed | Off | Rest state. Full startup sequence to reach COMPRESSING. |
| **WARM** | Running | Engaged | Off | One PCSV toggle away from compressing. No ramp cost. |
| **COMPRESSING** | Running | Engaged | Cycling | Pressure rising. |

### Command end states

| Command | `on_target` | End state | Description |
|---|---|---|---|
| `COMPRESS(1)` | `ON_TARGET_SUCCEED(0)` | OFF | One-shot: ramp up → compress to target → ordered shutdown |
| `COMPRESS(1)` | `ON_TARGET_HOLD(1)` | WARM (looping) | Compress to target → go WARM → re-engage when outlet drops below `reenable_pressure_bar` → repeat. Holds open until STOP. Used for interstage band-control in SYNC. |
| `STOP(2)` | — | OFF | Ordered shutdown: PCSV off → HPU close → ramp down → inlet close |
| `FORCE_STOP(3)` | — | OFF | Immediate halt — all outputs off simultaneously, no ramp-down wait |

---

## Package structure

```
include/hyfleet_booster/
  booster_node.hpp          Public node declaration
src/
  booster_node.cpp          Lifecycle callbacks, params, BT wiring, tick timer
  booster_action.cpp        Action server: goal/cancel/feedback/result protocol
  include/
    booster_action.hpp      BoosterAction class (internal)
    booster_bt_actions.hpp  BT action node declarations (StartVFD, StopVFD, SetPCSV,
                            HoldPCSV, ControlSV)
    booster_bt_conditions.hpp  BT node declarations (InletPressureStable, VFDAtSpeed,
                               VFDStopped, OutletAtPressure, PressureBelowThreshold,
                               OnTargetIs, InletGuard, InletPressureSafe,
                               LogCompressionStart)
    booster_telemetry_cache.hpp  Thread-safe telemetry cache (owned by BoosterNode,
                                 shared via blackboard)
    booster_bt_actions.cpp
    booster_bt_conditions.cpp
trees/
  start_tree.xml            Startup sequence: inlet SV → InletPressureStable → VFD →
                            HPU — leaves booster in WARM state
  compress_tree.xml         COMPRESS — startup → compress; on_target selects once/hold;
                            on_inlet_starve selects abort/pause; ordered shutdown on exit
  stop_tree.xml             STOP — disable PCSV, close HPU, ramp down VFD, close inlet
  safe_stop_tree.xml        FORCE_STOP — immediate stop, all outputs slammed off in Parallel
```

---

## How the state machine maps to BT nodes

The original `Booster::update()` was a hand-rolled switch over `CompressionPhaseState`.
Each case becomes a BT node. Nothing is lost — the logic moves from C++ into the XML
where it can be read, reordered, and extended without recompiling.

### State → node type mapping

| Archive state | BT node | Type |
|---|---|---|
| `WAIT_COMMAND` | tree entry point — starts when goal arrives | — |
| `RAMP_VFD_UP` | `StartVFD` | Action |
| `VFD_DELAY` | `Sleep msec` | Built-in |
| `WAIT_RAMP` | `VFDAtSpeed` | StatefulActionNode (Wait) |
| `WAIT_STABILIZATION` | `Sleep msec` | Built-in |
| `ENERGISE_VALVE` | `ControlSV` (HPU open) | Action |
| `PCSV_ON` (command) | `SetPCSV` (enable) | RosServiceNode — fires PCSV-on, returns SUCCESS when PLC confirms |
| PCSV-on state owner | `HoldPCSV` | StatefulActionNode — stays RUNNING indefinitely; fires PCSV-off in `onHalted()` on every exit path |
| outlet fill wait | `OutletAtPressure` | StatefulActionNode (Wait) |
| outlet droop wait | `PressureBelowThreshold` | StatefulActionNode (Wait) — ON_TARGET_HOLD only |
| re-engage / hold routing | `OnTargetIs` | ConditionNode (Gate) |
| inlet loss monitor | `InletGuard` | StatefulActionNode — ABORT mode: fails tree; PAUSE mode: stays RUNNING (halts PCSV), resumes on recovery |
| `PCSV_OFF` | triggered by halting `HoldPCSV` | (via onHalted) |
| `DEENERGISE_VALVE` | `ControlSV` (HPU close) | Action |
| `RAMP_VFD_DOWN` | `StopVFD` | Action |
| `WAIT_STOP` | `VFDStopped` | StatefulActionNode (Wait) |
| inlet safety monitor | `InletPressureSafe` | ConditionNode (Gate) |

### Action nodes — `RosServiceNode`

All hardware commands go through **one `BoosterCmd` service per instance**:
`/low_booster/booster_cmd` and `/high_booster/booster_cmd`.
The ADS bridge advertises these services and translates to the PLC wire format.

| Archive command | BT node | `cmd` constant | Key ports |
|---|---|---|---|
| `commands.vfd` (start) | `StartVFD` | `START_VFD=1` | `service_name`, `speed_rpm` |
| `commands.vfd` (stop) | `StopVFD` | `STOP_VFD=2` | `service_name` |
| `commands.pcsv` (command) | `SetPCSV` | `SET_PCSV=3` | `service_name`, `enable`, `cpm` — returns SUCCESS when PLC confirms |
| PCSV state holder | `HoldPCSV` | `SET_PCSV=3` | `service_name` — stays RUNNING; fires PCSV-off async via `onHalted()` |
| `commands.sv` (any valve) | `ControlSV` | `CONTROL_SV=4` | `service_name`, `sv_index`, `enable` |

### Telemetry nodes — shared cache

`BoosterNode` owns a single `rclcpp::Subscription<CompressorTelemetry>` (created at configure,
destroyed at cleanup). The callback writes to a `BoosterTelemetryCache` (thread-safe, mutex-guarded).
The cache `shared_ptr` is placed on the blackboard once at configure; all BT nodes and the tick loop
call `cache->latest()` to read. No BT node touches a subscription.

All telemetry comes from **one topic**: `compressor_telemetry`
(type `mserve_interfaces/msg/CompressorTelemetry`).
Pressure values are in `pt_bar[]`; VFD speed in `vfd_speed_rpm[]`; VFD state in `vfd_state[]`.
Indices are configured per instance via ROS params.

**Wait nodes (`BT::StatefulActionNode`)** — block until target state reached; may return RUNNING.

| Archive check | BT node | Field | Key ports |
|---|---|---|---|
| `is_inlet_stable()` | `InletPressureStable` | `pt_bar[inlet_pt_index]` | `inlet_pt_index`, `stabilization_samples`, `stability_tolerance`, `stability_timeout_ms` |
| `vfd_speed ≈ target` | `VFDAtSpeed` | `vfd_speed_rpm[vfd_index]` | `vfd_index`, `target_speed`, `ramp_tolerance`, `ramp_timeout_ms` |
| `outlet_pressure >= target` | `OutletAtPressure` | `pt_bar[outlet_pt_index]` | `outlet_pt_index`, `target_pressure` |
| `vfd_speed ≈ 0 && state != RUNNING` | `VFDStopped` | `vfd_speed_rpm[vfd_index]`, `vfd_state[vfd_index]` | `vfd_index`, `stop_threshold`, `stop_timeout_ms` |
| outlet dropped below re-enable threshold | `PressureBelowThreshold` | `pt_bar[pt_index]` | `pt_index`, `threshold_bar` OR `reenable_pressure_bar`, `timeout_ms` (0 = no limit) |
| inlet starvation / recovery | `InletGuard` | `pt_bar[inlet_pt_index]` | `inlet_pt_index`, `on_inlet_starve`, `inlet_starve_bar`, `inlet_resume_bar` |

**Gate nodes (`BT::ConditionNode`)** — instantaneous check; SUCCESS or FAILURE only, never RUNNING.

| Archive check | BT node | Field | Key ports |
|---|---|---|---|
| `inlet_pressure >= safe_pressure` | `InletPressureSafe` | `pt_bar[inlet_pt_index]` | `inlet_pt_index`, `safe_pressure` |
| on_target routing | `OnTargetIs` | blackboard `on_target` | `value` — SUCCESS if `on_target == value` |

### Built-in nodes — no C++ needed

| Archive timer | BT node | Blackboard key |
|---|---|---|
| `vfd_delay_` (cycle count) | `Sleep msec` | `vfd_delay_ms` |
| `wait_stabilization_` (cycle count) | `Sleep msec` | `vfd_stabilization_ms` |

---

## XML tree structure (compress_tree.xml)

```xml
<Sequence name="startup_and_compress">

  <SubTree ID="Start" _autoremap="true"/>

  <ReactiveSequence name="compress_loop">

    <InletGuard inlet_pt_index="{inlet_pt_index}"
                on_inlet_starve="{on_inlet_starve}"
                inlet_starve_bar="{inlet_starve_bar}"
                inlet_resume_bar="{inlet_resume_bar}"/>

    <Fallback>

      <!-- ON_TARGET_SUCCEED: compress once, return SUCCESS to coordinator -->
      <Sequence>
        <OnTargetIs value="0"/>
        <Parallel success_count="1" failure_count="1">
          <Sequence>
            <SetPCSV service_name="{service_name}" enable="true" cpm="{cpm}"/>
            <LogCompressionStart outlet_pt_index="{outlet_pt_index}" target_pressure="{target_pressure}"/>
            <HoldPCSV service_name="{service_name}"/>
          </Sequence>
          <OutletAtPressure outlet_pt_index="{outlet_pt_index}" target_pressure="{target_pressure}"/>
        </Parallel>
      </Sequence>

      <!-- ON_TARGET_HOLD: compress → PCSV-off → wait for droop → repeat -->
      <KeepRunningUntilFailure>
        <Sequence name="maintain_cycle">
          <Parallel success_count="1" failure_count="1">
            <Sequence>
              <SetPCSV service_name="{service_name}" enable="true" cpm="{cpm}"/>
              <LogCompressionStart outlet_pt_index="{outlet_pt_index}" target_pressure="{target_pressure}"/>
              <HoldPCSV service_name="{service_name}"/>
            </Sequence>
            <OutletAtPressure outlet_pt_index="{outlet_pt_index}" target_pressure="{target_pressure}"/>
          </Parallel>
          <PressureBelowThreshold pt_index="{outlet_pt_index}"
                                   reenable_pressure_bar="{reenable_pressure_bar}"
                                   timeout_ms="0"/>
        </Sequence>
      </KeepRunningUntilFailure>

    </Fallback>
  </ReactiveSequence>

</Sequence>
```

**PCSV ownership model**: `SetPCSV` is a pure PLC command — it sends PCSV-on and returns SUCCESS
when the PLC confirms. `HoldPCSV` is the state owner — it stays RUNNING indefinitely and fires
PCSV-off asynchronously in `onHalted()` on every exit path (target reached, inlet pause, goal abort).
This separation means the PCSV is always turned off, regardless of how the Parallel exits.

**`InletGuard` in `ReactiveSequence`**: when inlet pressure drops below `inlet_starve_bar` in PAUSE
mode, `InletGuard` returns RUNNING — the `ReactiveSequence` halts the Fallback subtree, which halts
`HoldPCSV`, which fires PCSV-off. When inlet recovers above `inlet_resume_bar`, `InletGuard` returns
SUCCESS and the Parallel restarts, firing PCSV-on again.

Config-derived `{key}` values are written to the blackboard at configure time.
`{target_pressure}`, `{speed_rpm}`, `{cpm}`, `{on_target}`, `{on_inlet_starve}`, `{inlet_starve_bar}`,
and `{inlet_resume_bar}` are written when a goal arrives.

---

## Parameters

All params follow one rule: **not changeable while the node is ACTIVE**. `on_parameters`
rejects any change in that state. Changes made while INACTIVE or UNCONFIGURED take effect
on the next `on_configure` call.

Two values are derived, not params:
- **Service name**: `/{node_name}/booster_cmd` — architecture contract
- **Telemetry topic**: `compressor_telemetry` — architecture contract

Hardware speed and CPM ceilings are `constexpr` in `include/hyfleet_booster/booster_limits.hpp`
(not params — a constant cannot be overridden at all). `target_pressure`, `cpm`, `speed_rpm`,
`on_target`, `on_inlet_starve`, `inlet_starve_bar`, and `inlet_resume_bar` are **goal fields**,
not params — they arrive with the `ControlBooster` goal and are written to the blackboard at
goal-accept time.

### Hardware mapping

Set per instance in the launch file. These index into `CompressorTelemetry` arrays.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `vfd_index` | int | 0 | `vfd_speed_rpm[i]`, `vfd_state[i]` — low=0, high=1 |
| `inlet_pt_index` | int | 0 | Inlet pressure — `pt_bar[x]` |
| `outlet_pt_index` | int | 1 | Outlet pressure — `pt_bar[x]` |
| `hyd_primer_pt_index` | int | 2 | Hydraulic primer pressure — `pt_bar[x]` |
| `hyd_a_pt_index` | int | 3 | Hydraulic circuit A pressure — `pt_bar[x]` |
| `hyd_b_pt_index` | int | 4 | Hydraulic circuit B pressure — `pt_bar[x]` |
| `coolant_pt_index` | int | 5 | Coolant pressure — `pt_bar[x]` |
| `inlet_tt_index_1` | int | 0 | Inlet temperature sensor 1 — `tt_celsius[x]` |
| `inlet_tt_index_2` | int | 1 | Inlet temperature sensor 2 — `tt_celsius[x]` |
| `outlet_tt_index_1` | int | 2 | Outlet temperature sensor 1 — `tt_celsius[x]` |
| `outlet_tt_index_2` | int | 3 | Outlet temperature sensor 2 — `tt_celsius[x]` |
| `outlet_tt_index_3` | int | 4 | Outlet temperature sensor 3 — `tt_celsius[x]` |
| `ps_lhs_index` | int | 0 | End-of-travel position switch LHS — `ps[x]` |
| `ps_rhs_index` | int | 1 | End-of-travel position switch RHS — `ps[x]` |
| `inlet_sv_index` | int | 0 | Inlet solenoid valve — `sv[x]` |
| `hpu_sv_index` | int | 1 | HPU solenoid valve — `sv[x]` |

### Operational

Tuned during commissioning. `target_pressure`, `cpm`, `speed_rpm`, `on_target`,
`on_inlet_starve`, `inlet_starve_bar`, and `inlet_resume_bar` are **not params** —
they arrive with the `ControlBooster` goal. Speed/CPM hardware ceilings are `constexpr` in
`booster_limits.hpp`, not params.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `min_pressure_bar` | double | 35.0 | Minimum valid goal pressure (bar) |
| `max_pressure_bar` | double | 350.0 | Maximum valid goal pressure (bar) |
| `safe_pressure` | double | 25.0 | Minimum inlet pressure to run safely (bar) |
| `target_deadband` | double | 0.5 | Pressure deadband around target (bar) |
| `stability_tolerance` | double | 0.05 | Max variation for inlet stability check (bar) |
| `min_temp_inlet` | double | 0.0 | Minimum inlet temperature (°C) |
| `max_temp_inlet` | double | 50.0 | Maximum inlet temperature (°C) |
| `min_temp_outlet` | double | 0.0 | Minimum outlet temperature (°C) |
| `max_temp_outlet` | double | 190.0 | Maximum outlet temperature (°C) |
| `vfd_delay_ms` | int | 2000 | Wait after VFD start before checking speed (ms) |
| `vfd_stabilization_ms` | int | 1000 | Wait after VFD at speed before opening HPU (ms) |
| `stability_timeout_ms` | int | 10000 | Max time for inlet pressure to stabilise before startup (ms) |
| `ramp_timeout_ms` | int | 30000 | Max time for VFD to reach target speed (ms) |
| `stop_timeout_ms` | int | 15000 | Max time for VFD to reach stopped state (ms) |
| `ramp_tolerance` | double | 25.0 | Band around target — VFD considered at speed (rpm) |
| `stop_threshold` | double | 25.0 | Below this — VFD considered stopped (rpm) |
| `stabilization_samples` | int | 3 | Rolling window depth for `InletPressureStable` |
| `reenable_pressure_bar` | double | 230.0 | Absolute outlet pressure below which `ON_TARGET_HOLD` re-engages PCSV (bar). Fixed machine property — not a goal field. |

---

## Running

```bash
# Full stack (recommended)
source install/setup.bash
ros2 launch hyfleet_sim hyfleet_sim.launch.py

# Send a test goal (compress once)
ros2 action send_goal /low_booster/control_booster \
  mserve_interfaces/action/ControlBooster \
  "{command: 1, on_target: 0, target_pressure: 350.0}"

# Tune a commissioning param (node must not be ACTIVE)
ros2 lifecycle set /low_booster deactivate
ros2 lifecycle set /low_booster cleanup
ros2 param set /low_booster reenable_pressure_bar 220.0
ros2 lifecycle set /low_booster configure
ros2 lifecycle set /low_booster activate
```

---

## Action interface — `ControlBooster`

```
Goal:
  uint8 COMPRESS   = 1
  uint8 STOP       = 2
  uint8 FORCE_STOP = 3

  uint8 ON_TARGET_SUCCEED = 0   ← compress once; return SUCCESS at target
  uint8 ON_TARGET_HOLD    = 1   ← maintain band; loop until preempted then STOP

  uint8 INLET_STARVE_ABORT = 0  ← InletGuard fails the tree if inlet drops
  uint8 INLET_STARVE_PAUSE = 1  ← InletGuard blocks (PCSV off) while starved; resumes on recovery

  uint8   command
  float64 target_pressure     ← validated against min/max_pressure_bar on goal-accept
  float64 cpm                 ← coordinator-decided; validated against CPM_MAX constexpr
  float64 speed_rpm           ← coordinator-decided; validated against SPEED_MAX_RPM constexpr
  uint8   on_target           ← 0 = succeed at target, 1 = hold band
  uint8   on_inlet_starve     ← 0 = abort, 1 = pause
  float64 inlet_starve_bar    ← starvation threshold (-1 = not used)
  float64 inlet_resume_bar    ← recovery threshold (-1 = not used)

Result:
  bool accepted
  string message

Feedback:
  float64 pressure
  float64 percent_complete
```

`ON_TARGET_HOLD` compresses to target pressure then enters a maintain loop: PCSV off (WARM),
wait until outlet drops below `reenable_pressure_bar`, re-engage PCSV, compress back to target,
repeat. VFD stays running throughout — no ramp cost on re-engage. Used for interstage
band-control in SYNC mode. The re-enable threshold is a fixed operational param
(`reenable_pressure_bar`, absolute bar), not a goal field.

Goal fields are written to the blackboard at goal-accept and consumed by the BT tree.
They are not ROS parameters.

---

## Dependencies

- `behaviortree_cpp` — `ros-jazzy-behaviortree-cpp` (apt)
- `behaviortree_ros2` — source build (BehaviorTree/BehaviorTree.ROS2)
- `rclcpp_action`, `rclcpp_lifecycle`
- `mserve_interfaces` — `ControlBooster.action`, `BoosterCmd.srv`, `CompressorTelemetry.msg`
- `mserve_utils` — `get_or_declare_param` helpers (runtime param declaration + warn on default)

# hyfleet_booster

ROS 2 lifecycle node for a single hydrogen compression booster stage.
The same binary is launched twice — as `low_booster` and `high_booster` — with
different parameters driving all differences between instances.

Each instance owns an `rclcpp_action` server and an internal BehaviorTree.CPP tree.
The tree is the state machine. No hand-rolled switch statements.

---

## What it does

- Accepts `ControlBooster` action goals: `START`, `START_IDLE`, `STOP`, `SAFE_STOP`
- Loads four XML trees at configure time — one per command
- Ticks the active tree on a 100 ms wall timer while a goal is active
- Publishes outlet pressure + percent_complete feedback each tick
- Succeeds or aborts the goal when the tree resolves
- Lifecycle managed: configure / activate / deactivate / cleanup / shutdown

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
    booster_bt_actions.hpp  BT action node declarations (StartVFD, StopVFD, SetPCSV, ControlSV)
    booster_bt_conditions.hpp  BT condition node declarations (InletPressureStable, VFDAtSpeed,
                               OutletAtPressure, InletPressureSafe)
    booster_bt_actions.cpp
    booster_bt_conditions.cpp
trees/
  start_tree.xml            START command — ramp up, stabilise, open HPU, enable PCSV
  start_idle_tree.xml       START_IDLE — same as start but holds at target pressure
  stop_tree.xml             STOP — disable PCSV, close HPU, ramp down VFD, close inlet
  stop_force_tree.xml       SAFE_STOP — immediate stop, no ramp-down wait
```

---

## How the archive state machine maps to BT nodes

The original `Booster::update()` was a hand-rolled switch over `CompressionPhaseState`.
Each case becomes a BT node. Nothing is lost — the logic moves from C++ into the XML
where it can be read, reordered, and extended without recompiling.

### State → node type mapping

| Archive state | BT node | Type |
|---|---|---|
| `WAIT_COMMAND` | tree entry point — starts when goal arrives | — |
| `RAMP_VFD_UP` | `StartVFD` | Action |
| `VFD_DELAY` | `Wait` | Built-in |
| `WAIT_RAMP` | `VFDAtSpeed` | Condition |
| `WAIT_STABILIZATION` | `Wait` | Built-in |
| `ENERGISE_VALVE` | `ControlSV` (HPU open) | Action |
| `PCSV_ON` | `SetPCSV` (enable) | Action |
| `PCSV_OFF` | `SetPCSV` (disable) | Action |
| `DEENERGISE_VALVE` | `ControlSV` (HPU close) | Action |
| `RAMP_VFD_DOWN` | `StopVFD` | Action |
| `WAIT_STOP` | `VFDAtSpeed` (zero + stop threshold) | Condition |

### Action nodes — `RosServiceNode`

All hardware commands go through **one `BoosterCmd` service per instance**:
`/low_booster/booster_cmd` and `/high_booster/booster_cmd`.
The ADS bridge advertises these services and translates to the PLC wire format.
BT nodes set named fields — no payload encoding in the BT layer.

| Archive command | BT node | `cmd` constant | Key ports |
|---|---|---|---|
| `commands.vfd` (start) | `StartVFD` | `START_VFD=1` | `service_name`, `speed_rpm` |
| `commands.vfd` (stop) | `StopVFD` | `STOP_VFD=2` | `service_name` |
| `commands.pcsv` | `SetPCSV` | `SET_PCSV=3` | `service_name`, `enable`, `cpm` |
| `commands.sv` (any valve) | `ControlSV` | `CONTROL_SV=4` | `service_name`, `device_id`, `enable` |

### Condition nodes — `RosTopicSubNode`

All telemetry comes from **one topic**: `compressor_telemetry`
(type `mserve_interfaces/msg/CompressorTelemetry`).
Pressure values are in `hbu_pt_bar[]`; VFD speed is in `vfd_speed_rpm[]`.
Indices are configured per instance via ROS params — the same node type works for both boosters.

| Archive check | BT node | Field | Key ports |
|---|---|---|---|
| `is_inlet_stable()` | `InletPressureStable` | `hbu_pt_bar[inlet_pt_index]` | `topic_name`, `inlet_pt_index` |
| `vfd_speed >= target - deadband` | `VFDAtSpeed` | `vfd_speed_rpm[vfd_index]` | `topic_name`, `vfd_index`, `target_speed` |
| `outlet_pressure >= target` | `OutletAtPressure` | `hbu_pt_bar[outlet_pt_index]` | `topic_name`, `outlet_pt_index`, `target_pressure` |
| `inlet_pressure >= safe_pressure` | `InletPressureSafe` | `hbu_pt_bar[inlet_pt_index]` | `topic_name`, `inlet_pt_index`, `safe_pressure` |

### Built-in nodes — no C++ needed

| Archive timer | BT node | Blackboard key |
|---|---|---|
| `vfd_delay_` (cycle count) | `Wait` | `vfd_delay_s` |
| `wait_stabilization_` (cycle count) | `Wait` | `stabilization_s` |

---

## XML tree structure (start_tree.xml)

```xml
<Parallel failure_threshold="1">

  <Sequence name="startup">
    <ControlSV service_name="{service_name}" device_id="{inlet_sv_id}" enable="true"/>
    <InletPressureStable topic_name="{telemetry_topic}" inlet_pt_index="{inlet_pt_index}"/>
    <StartVFD service_name="{service_name}" speed_rpm="{speed_rpm}"/>
    <Wait delay="{vfd_delay_s}"/>
    <VFDAtSpeed topic_name="{telemetry_topic}" vfd_index="{vfd_index}" target_speed="{speed_rpm}"/>
    <Wait delay="{stabilization_s}"/>
    <ControlSV service_name="{service_name}" device_id="{hpu_sv_id}" enable="true"/>
    <SetPCSV service_name="{service_name}" enable="true" cpm="{cpm}"/>
    <OutletAtPressure topic_name="{telemetry_topic}" outlet_pt_index="{outlet_pt_index}"
                      target_pressure="{target_pressure}"/>
  </Sequence>

  <InletPressureSafe topic_name="{telemetry_topic}" inlet_pt_index="{inlet_pt_index}"
                     safe_pressure="{safe_pressure}"/>

</Parallel>
```

Config-derived `{key}` values are written to the blackboard at configure time.
`{target_pressure}`, `{speed_rpm}`, and `{cpm}` are written when a goal arrives.

---

## Parameters

All params follow one rule: **not changeable while the node is ACTIVE**. `on_parameters`
rejects any change in that state. Changes made while INACTIVE or UNCONFIGURED take effect
on the next `on_configure` call. See [architecture.md](../hyfleet_compressor/docs/architecture.md)
for the full decision rationale.

Two values are derived, not params:
- **Service name**: `/{node_name}/booster_cmd` — architecture contract
- **Telemetry topic**: `compressor_telemetry` — architecture contract

Hardware speed and CPM ceilings are `constexpr` in `include/hyfleet_booster/booster_limits.hpp`
(not params — a constant cannot be overridden at all). `target_pressure`, `cpm`, and `speed_rpm`
are **goal fields**, not params — they arrive with the `ControlBooster` goal and are written to
the blackboard at goal-accept time.

### Hardware mapping

Set per instance in the launch file. These index into `CompressorTelemetry` arrays.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `vfd_index` | int | 0 | `vfd_speed_rpm[i]` — low=0, high=1 |
| `heater_index` | int | 0 | `heater_on[i]` — low=0, high=1 |
|
| `inlet_pt_index` | int | 0 | Inlet pressure — `pt_bar[x]` |
| `outlet_pt_index` | int | 1 | Outlet pressure — `pt_bar[x]` |
| `hyd_primer_pt_index` | int | 2 | Hydraulic primer pressure — `pt_bar[x]` |
| `hyd_a_pt_index` | int | 3 | Hydraulic circuit A pressure — `pt_bar[x]` |
| `hyd_b_pt_index` | int | 4 | Hydraulic circuit B pressure — `pt_bar[x]` |
| `coolant_pt_index` | int | 5 | Coolant pressure — `pt_bar[x]` |
|
| `inlet_tt_index_1` | int | 0 | Inlet temperature sensor 1 — `tt_celsius[x]` |
| `inlet_tt_index_2` | int | 1 | Inlet temperature sensor 2 — `tt_celsius[x]` |
| `outlet_tt_index_1` | int | 2 | Outlet temperature sensor 1 — `tt_celsius[x]` |
| `outlet_tt_index_2` | int | 3 | Outlet temperature sensor 2 — `tt_celsius[x]` |
| `outlet_tt_index_3` | int | 4 | Outlet temperature sensor 3 — `tt_celsius[x]` |
|
| `ps_lhs_index` | int | 0 | End-of-travel position switch LHS — `ps[x]` |
| `ps_rhs_index` | int | 1 | End-of-travel position switch RHS — `ps[x]` |
|
| `inlet_sv_id` | string | "inlet_sv" | Inlet solenoid valve device ID |
| `hpu_sv_id` | string | "hpu_sv" | HPU solenoid valve device ID |

### Operational

Tuned during commissioning. `target_pressure`, `cpm`, and `speed_rpm` are **not params** —
they arrive with the `ControlBooster` goal. Speed/CPM hardware ceilings are `constexpr` in
`booster_limits.hpp`, not params.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `ramp_tolerance` | double | 25.0 | Band around target — VFD considered at speed (rpm) |
| `stop_threshold` | double | 25.0 | Below this — VFD considered stopped (rpm) |
|
| `min_pressure_bar` | double | 35.0 | Minimum valid goal pressure (bar) |
| `max_pressure_bar` | double | 350.0 | Maximum valid goal pressure (bar) |
| `safe_pressure` | double | 25.0 | Minimum inlet pressure to run safely (bar) |
| `target_deadband` | double | 0.5 | Pressure deadband around target (bar) |
|
| `min_temp_inlet` | double | 0.0 | Minimum inlet temperature (°C) |
| `max_temp_inlet` | double | 50.0 | Maximum inlet temperature (°C) |
| `min_temp_outlet` | double | 0.0 | Minimum outlet temperature (°C) |
| `max_temp_outlet` | double | 190.0 | Maximum outlet temperature (°C) |
|
| `stability_tolerance` | double | 0.05 | Max variation for inlet stability check (bar) |
| `vfd_delay_ms` | int | 2000 | Wait after VFD start before checking speed (ms) |
| `vfd_stabilization_ms` | int | 1000 | Wait after VFD at speed before opening HPU (ms) |
| `stabilization_samples` | int | 3 | Rolling window depth for `InletPressureStable` |

---

## Running

```bash
# Full stack (recommended)
source install/setup.bash
ros2 launch mserve_launch mserve_min.launch.py

# Send a test goal
ros2 action send_goal /low_booster/control_booster \
  mserve_interfaces/action/ControlBooster \
  "{command: 1, target_pressure: 350.0}"

# Tune a commissioning param (node must not be ACTIVE)
ros2 lifecycle set /low_booster deactivate
ros2 lifecycle set /low_booster cleanup
ros2 param set /low_booster target_speed 1600.0
ros2 lifecycle set /low_booster configure
ros2 lifecycle set /low_booster activate
```

---

## Action interface — `ControlBooster`

```
Goal:
  uint8 command          START=1, START_IDLE=2, STOP=3, SAFE_STOP=4
  float64 target_pressure   ← validated against min/max_pressure_bar on goal-accept
  float64 cpm               ← coordinator-decided; validated against CPM_MAX constexpr
  float64 speed_rpm         ← coordinator-decided; validated against SPEED_MAX_RPM constexpr

Result:
  bool accepted
  string message

Feedback:
  float64 pressure
  float64 percent_complete
```

`START_IDLE` boosts to target pressure then holds with poppet valve engaged.
VFD stays available for rapid restart. Used during SYNC mode when the coordinator
will issue another START shortly after.

Goal fields `target_pressure`, `cpm`, and `speed_rpm` are written to the blackboard
at goal-accept and consumed by the BT tree. They are not ROS parameters.

---

## Dependencies

- `behaviortree_cpp` — `ros-jazzy-behaviortree-cpp` (apt)
- `behaviortree_ros2` — source build (BehaviorTree/BehaviorTree.ROS2)
- `rclcpp_action`, `rclcpp_lifecycle`
- `mserve_interfaces` — `ControlBooster.action`, `BoosterCmd.srv`, `CompressorTelemetry.msg`
- `mserve_utils` — `get_or_declare_param` helpers (runtime param declaration + warn on default)

# hyfleet_booster

ROS 2 lifecycle node for a single hydrogen compression booster stage.
The same binary is launched twice — as `low_booster` and `high_booster` — with
different config files driving all differences.

Each instance owns an `rclcpp_action` server and an internal BehaviorTree.CPP tree.
The tree is the state machine. No hand-rolled switch statements.

---

## What it does

- Accepts `ControlBooster` action goals: `START`, `START_IDLE`, `STOP`, `SAFE_STOP`
- Loads `trees/booster.xml` at configure time — edit the XML to change behaviour
- Ticks the tree on a 100ms wall timer while a goal is active
- Publishes pressure feedback each tick
- Succeeds or aborts the goal when the tree resolves
- Lifecycle managed: configure/activate/deactivate/cleanup/shutdown

---

## Package structure

```
include/hyfleet_booster/
  booster_node.hpp        Public node declaration
src/
  booster_node.cpp        Lifecycle callbacks, BT wiring, tick timer
  booster_action.cpp      Action server: goal/cancel/feedback/result protocol
  include/
    booster_action.hpp    BoosterAction class (internal)
  trees/
    booster.xml           BT tree — edit here to change startup/shutdown sequence
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
| `RAMP_VFD_UP` | `SetVFD` (start) | Action |
| `VFD_DELAY` | `Wait` | Built-in |
| `WAIT_RAMP` | `VFDAtSpeed` | Condition |
| `WAIT_STABILIZATION` | `Wait` | Built-in |
| `ENERGISE_VALVE` | `SetSolenoidValve` (HPU on) | Action |
| `PCSV_ON` | `SetPCSV` (enable) | Action |
| `PCSV_OFF` | `SetPCSV` (disable) | Action |
| `DEENERGISE_VALVE` | `SetSolenoidValve` (HPU off) | Action |
| `RAMP_VFD_DOWN` | `SetVFD` (stop) | Action |
| `WAIT_STOP` | `VFDAtSpeed` (zero threshold) | Condition |

### Action nodes — `RosServiceNode`

All hardware commands go through **one `BoosterCmd` service per instance**:
`/low_booster/booster_cmd` and `/high_booster/booster_cmd`.
The ADS bridge advertises these services and translates to the PLC wire format.
BT nodes set named fields — no payload encoding in the BT layer.

| Archive command | BT node | `cmd` | Key fields |
|---|---|---|---|
| `commands.vfd` (start) | `StartVFD` | `START_VFD=1` | `speed_rpm` |
| `commands.vfd` (stop) | `StopVFD` | `STOP_VFD=2` | — |
| `commands.pcsv` | `SetPCSV` | `SET_PCSV=3` | `enable`, `cpm` |
| `commands.sv` (any valve) | `ControlSV` | `CONTROL_SV=4` | `device_id`, `enable` |

### Condition nodes — `RosTopicSubNode`

All telemetry comes from **one topic**: `compressor_telemetry`
(type `mserve_interfaces/msg/CompressorTelemetry`).
Pressure values are in `hbu_pt_bar[]` arrays; VFD speed is a direct field.
Indices are passed in via BT ports so the same node type works for both boosters.

| Archive check | BT node | Field | Config index |
|---|---|---|---|
| `is_inlet_stable()` | `InletPressureStable` | `hbu_pt_bar[inlet_pt_index]` | low=0, high=1 |
| `vfd_speed >= target` | `VFDAtSpeed` | `vfd1_speed_rpm` / `vfd2_speed_rpm` | low=1, high=2 |
| `outlet_pressure >= target` | `OutletAtPressure` | `hbu_pt_bar[outlet_pt_index]` | low=7, high=2 |
| `inlet_pressure < threshold` | `InletPressureSafe` | `hbu_pt_bar[inlet_pt_index]` | low=0, high=1 |

### Built-in nodes — no C++ needed

| Archive timer | BT node | Config param |
|---|---|---|
| `vfd_delay_` (cycle count) | `Wait msec="..."` | `vfd_delay_ms` |
| `wait_stabilization_` (cycle count) | `Wait msec="..."` | `stabilization_ms` |

---

## What the XML tree looks like (Stage 2 target)

```xml
<Parallel failure_threshold="1">

  <!-- Operation sequence -->
  <Sequence name="startup_and_run">
    <SetSolenoidValve device_id="{inlet_sv_id}" state="true"/>
    <InletPressureStable/>
    <SetVFD device_id="{vfd_id}" enable="true" speed_rpm="{vfd_target_speed}"/>
    <Wait msec="{vfd_delay_ms}"/>
    <VFDAtSpeed device_id="{vfd_id}" target_rpm="{vfd_target_speed}"/>
    <Wait msec="{stabilization_ms}"/>
    <SetSolenoidValve device_id="{hpu_sv_id}" state="true"/>
    <SetPCSV device_id="{pcsv_id}" enable="true" cpm="{pcsv_cpm}"/>
    <OutletAtPressure target="{target_pressure}"/>
  </Sequence>

  <!-- Safety monitor — runs every tick alongside operation -->
  <Sequence name="safety">
    <InletPressureSafe min_pressure="{off_inlet_pressure}"/>
    <Inverter><IsForceStopRequested/></Inverter>
  </Sequence>

</Parallel>
```

When `safety` returns FAILURE (unsafe inlet or force stop), the `Parallel` node
fails immediately, triggering the shutdown sequence.

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
```

---

## Action interface — `ControlBooster`

```
Goal:
  uint8 command          START=1, START_IDLE=2, STOP=3, SAFE_STOP=4
  float64 target_pressure

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

---

## Dependencies

- `behaviortree_cpp` — `ros-jazzy-behaviortree-cpp` (apt)
- `behaviortree_ros2` — source build (BehaviorTree/BehaviorTree.ROS2)
- `rclcpp_action`, `rclcpp_lifecycle`
- `mserve_interfaces` — `ControlBooster.action`

# hyfleet_compressor

ROS 2 lifecycle coordinator node for the HyFleet compression module.
Sits between the orchestrator and the two booster nodes — routes `ControlCompressor`
goals to `low_booster`, `high_booster`, or both (SYNC) via `RosActionNode` BT calls.

Single binary, single instance: `hyfleet_compression`.
Brought up last — after both boosters are active.

Each instance owns an `rclcpp_action` server and an internal BehaviorTree.CPP tree.
The tree is the coordinator state machine. No hand-rolled routing logic.

---

## What it does

- Accepts `ControlCompressor` action goals: `START`, `STOP`, `SAFE_STOP`
- Routes to target: `LOW_BOOSTER`, `HIGH_BOOSTER`, or `SYNC_BOOSTERS`
- Loads three XML trees at configure time — one per command
- Ticks the active tree on a 100 ms wall timer while a goal is active
- Succeeds or aborts the goal when the tree resolves
- Lifecycle managed: configure / activate / deactivate / cleanup / shutdown

> **Stage 3 — in progress.** Trees currently contain `AlwaysSuccess` placeholders.
> `RosActionNode` wrappers (`BoostLow`, `BoostHigh`) and SYNC routing are next.

---

## Package structure

```
include/hyfleet_compressor/
  compressor_node.hpp       Public node declaration
  compressor_limits.hpp     Outer constexpr pressure bounds (per-target limits are params)
src/
  compressor_node.cpp       Lifecycle callbacks, params, BT wiring, tick timer
  compressor_params.cpp     Parameter declaration, loading, on_parameters callback
  compressor_action.cpp     Action server: goal/cancel/result protocol
  compressor_bt_nodes.cpp   BT node implementations (Stage 3+: BoostLow, BoostHigh)
  include/
    compressor_action.hpp   CompressorAction class (internal)
    compressor_bt_nodes.hpp BT node declarations (Stage 3: RosActionNode wrappers)
trees/
  start_tree.xml            START command — routes to LOW / HIGH / SYNC booster(s)
  stop_tree.xml             STOP command
  safe_stop_tree.xml        SAFE_STOP command — immediate halt, no ramp-down wait
```

---

## How it maps to the architecture

```
Orchestrator
    │  ControlCompressor goal (command, target, mode, target_pressure)
    ▼
CompressorNode  (hyfleet_compression)
    │  BT tree routes by target field on blackboard
    ├── LOW   → RosActionNode → /low_booster/control_booster
    ├── HIGH  → RosActionNode → /high_booster/control_booster
    └── SYNC  → Parallel
                  ├── RosActionNode → /low_booster/control_booster  (START_IDLE)
                  └── RosActionNode → /high_booster/control_booster (START)
```

The coordinator owns **mode → setpoint translation**: it reads mode (PERFORMANCE / ECO)
from the goal and writes `cpm` and `speed_rpm` to the blackboard from its profile params.
The booster nodes never see the mode — they only see the translated setpoints in the goal.

---

## Node pattern (identical to boosters)

```
LifecycleNode
  constructor   → declare_params()
  on_configure  → load_params(), create action server, build BT trees
  on_activate   → set accepting_goals = true
  on_deactivate → set accepting_goals = false, abort active goal
  on_cleanup    → destroy action server, destroy trees, reset factory
  on_parameters → reject while not UNCONFIGURED

  goal arrives  → validate command / target / target_pressure
                  write goal fields to blackboard
                  select tree, start 100 ms tick timer
  tick timer    → tree.tickOnce() → succeed / abort on completion
```

---

## Action interface — `ControlCompressor`

```
Goal:
  uint8 command          START=1, STOP=2, SAFE_STOP=3
  uint8 target           LOW_BOOSTER=1, HIGH_BOOSTER=2, SYNC_BOOSTERS=3
  uint8 mode             PERFORMANCE=1, ECO=2  (coordinator owns → setpoint translation)
  float64 target_pressure   ← validated against min/max_pressure_bar on goal-accept

Result:
  bool accepted
  string message

Feedback:
  float64 pressure
  float64 percent_complete
```

`target_pressure` is goal-only, never a param. `cpm` and `speed_rpm` are
coordinator-computed from profile params — they are written to the blackboard
and consumed by the `BoostLow` / `BoostHigh` `RosActionNode` wrappers as goal fields.

---

## Blackboard contents

Written at configure time (architecture contracts — not params):

| Key | Value |
|---|---|
| `low_booster_action` | `/low_booster/control_booster` |
| `high_booster_action` | `/high_booster/control_booster` |
| `service_name` | `/{node_name}/compressor_cmd` |

Written at configure time (from params):

| Key | Default | Description |
|---|---|---|
| `inlet_pt_index` | 0 | Inlet H2 pressure — `pt_bar[x]` |
| `interstage_pt_index` | 1 | Interstage H2 pressure — `pt_bar[x]` |
| `outlet_pt_index` | 2 | Final outlet H2 pressure — `pt_bar[x]` |
| `interstage_sv_index` | 0 | Interstage solenoid valve — `sv[x]` |
| `min_pressure_bar` | 35.0 | from param |
| `max_pressure_bar` | 900.0 | from param |

Written at goal-accept time (from goal):

| Key | Value |
|---|---|
| `command` | goal command (int) |
| `target` | goal target (int) |
| `mode` | goal mode (int) |
| `target_pressure` | bar |

Stage 3+: `cpm` and `speed_rpm` will be added at goal-accept once profile params are in.

---

## Parameters

All params follow one rule: **not changeable while the node is not UNCONFIGURED**.
`on_parameters` rejects any change otherwise. Changes take effect on the next `on_configure`.

### Hardware mapping (set in launch file)

Array indices into `CompressorTelemetry` fields for the system-level sensors the
coordinator monitors. Booster-local sensors (booster inlet/outlet PTs etc.) remain
params of the booster nodes, not the coordinator.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `inlet_pt_index` | int | 0 | System inlet H2 pressure — `pt_bar[x]` |
| `interstage_pt_index` | int | 1 | Interstage pressure (between low and high booster) — `pt_bar[x]` |
| `outlet_pt_index` | int | 2 | Final system outlet H2 pressure — `pt_bar[x]` |
| `interstage_sv_index` | int | 0 | Interstage solenoid valve — `sv[x]` |

### Operational

| Parameter | Type | Default | Description |
|---|---|---|---|
| `min_pressure_bar` | double | 35.0 | Minimum valid goal pressure (bar) |
| `max_pressure_bar` | double | 900.0 | Maximum valid goal pressure (bar) |

Stage 3+ will add mode profile params: `default_speed_rpm`, `eco_cpm`, `performance_cpm`.
See [architecture.md](docs/architecture.md) §6 for the mode → setpoint decision.

---

## Running

```bash
source install/setup.bash
ros2 launch mserve_launch mserve_min.launch.py

# Send a test goal (placeholder tree — succeeds immediately)
ros2 action send_goal /hyfleet_compression/control_compressor \
  mserve_interfaces/action/ControlCompressor \
  "{command: 1, target: 1, mode: 1, target_pressure: 350.0}"
```

Bringup order: `base → drivechain → low_booster → high_booster → hyfleet_compression`

---

## Dependencies

- `behaviortree_cpp` — `ros-jazzy-behaviortree-cpp` (apt)
- `behaviortree_ros2` — source build (BehaviorTree/BehaviorTree.ROS2)
- `rclcpp_action`, `rclcpp_lifecycle`
- `mserve_interfaces` — `ControlCompressor.action`, `ControlBooster.action`
- `mserve_utils` — `get_or_declare_param` helpers

See also: [architecture.md](docs/architecture.md), [todo.md](docs/todo.md)

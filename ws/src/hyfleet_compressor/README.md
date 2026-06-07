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
- Loads XML trees at configure time — one per command path
- Ticks the active tree on a 100 ms wall timer while a goal is active
- Succeeds or aborts the goal when the tree resolves
- Lifecycle managed: configure / activate / deactivate / cleanup / shutdown

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
  compressor_bt_nodes.cpp   BT node implementations (BoostLow, BoostHigh, ControlSV,
                            InterstageAboveBand)
  include/
    compressor_action.hpp   CompressorAction class (internal)
    compressor_bt_nodes.hpp BT node declarations
trees/
  parallel_low.xml          LOW_BOOSTER path — BoostLow(command={command}, ...)
  parallel_high.xml         HIGH_BOOSTER path — BoostHigh(command={command}, ...)
  sync.xml                  SYNC_BOOSTERS — two-phase interstage coordination
```

---

## How it maps to the architecture

```
Orchestrator
    │  ControlCompressor goal (command, target, mode, target_pressure)
    ▼
CompressorNode  (hyfleet_compression)
    │  BT tree routes by target field on blackboard
    ├── LOW   → BoostLow  → /low_booster/control_booster
    ├── HIGH  → BoostHigh → /high_booster/control_booster
    └── SYNC  → sync.xml
                  Phase 1: BoostLow(ON_TARGET_HOLD, interstage_target_bar)
                           InterstageAboveBand (wait for PT >= start_threshold)
                  Phase 2: ControlSV(interstage_sv, open)
                           BoostHigh(ON_TARGET_SUCCEED, target_pressure)
                           InletGuard on high booster pauses on interstage starvation
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

## SYNC two-phase tree

SYNC uses a single `ControlCompressor` goal and coordinates both boosters internally.

**Phase 1 — Build interstage (SV closed):**
The low booster compresses the interstage tank with `ON_TARGET_HOLD` at `interstage_target_bar`.
`InterstageAboveBand` polls `pt_bar[interstage_pt_index]` and blocks until pressure reaches
`interstage_start_threshold_bar`. The interstage SV stays closed throughout Phase 1 — pressure
builds on the low side of the SV before the connection is made.

**Phase 2 — Open SV, compress high stage:**
`ControlSV` opens the interstage SV. The high booster's inlet is now fed by the low booster.
`BoostHigh` compresses to `target_pressure` with `INLET_STARVE_PAUSE` — if interstage pressure
droops below `interstage_starvation_bar`, the high booster pauses (PCSV off) and waits for the
low booster to recover it above `interstage_start_threshold_bar`.

**Exit / abort:** `Parallel(success_count=1)` exits when the high booster hits target. The low
booster is halted by the Parallel, then both receive explicit `STOP(2)`. The interstage SV is
closed before stopping boosters in both success and abort paths.

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
| `interstage_pt_index` | 7 | Interstage H2 pressure — `pt_bar[x]` |
| `outlet_pt_index` | 2 | Final outlet H2 pressure — `pt_bar[x]` |
| `interstage_sv_index` | 4 | Interstage solenoid valve — `sv[x]` |
| `min_pressure_bar` | 35.0 | from param |
| `max_pressure_bar` | 900.0 | from param |
| `default_speed_rpm` | 1500.0 | from param |
| `performance_cpm` | 16.0 | from param |
| `eco_cpm` | 8.0 | from param |
| `interstage_target_bar` | 280.0 | Low booster hold target during SYNC Phase 1 |
| `interstage_start_threshold_bar` | 200.0 | Interstage PT threshold to open SV and start high booster |
| `interstage_starvation_bar` | 175.0 | Interstage PT threshold below which high booster pauses |
| `sync_overall_timeout_ms` | 3600000 | Wall-clock timeout for the entire SYNC operation |

Written at goal-accept time (from goal):

| Key | Value |
|---|---|
| `command` | goal command (int) |
| `target` | goal target (int) |
| `mode` | goal mode (int) |
| `target_pressure` | bar |
| `cpm` | from mode profile |
| `speed_rpm` | from mode profile |

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
| `interstage_pt_index` | int | 7 | Interstage pressure (low booster outlet / high booster inlet) — `pt_bar[x]` |
| `outlet_pt_index` | int | 2 | Final system outlet H2 pressure — `pt_bar[x]` |
| `interstage_sv_index` | int | 4 | Interstage solenoid valve — `sv[x]` |

### Operational

| Parameter | Type | Default | Description |
|---|---|---|---|
| `min_pressure_bar` | double | 35.0 | Minimum valid goal pressure (bar) |
| `max_pressure_bar` | double | 900.0 | Maximum valid goal pressure (bar) |
| `default_speed_rpm` | double | 1500.0 | VFD speed sent to boosters |
| `performance_cpm` | double | 16.0 | CPM at PERFORMANCE mode |
| `eco_cpm` | double | 8.0 | CPM at ECO mode |
| `interstage_target_bar` | double | 280.0 | Low booster hold target during SYNC Phase 1 (bar) |
| `interstage_start_threshold_bar` | double | 200.0 | Interstage PT threshold to open SV and start high booster (bar) |
| `interstage_starvation_bar` | double | 175.0 | Interstage PT threshold below which high booster pauses (bar) |
| `sync_overall_timeout_ms` | int | 3600000 | Wall-clock timeout for entire SYNC operation (ms) |

---

## Running

```bash
source install/setup.bash
ros2 launch hyfleet_sim hyfleet_sim.launch.py

# LOW single booster
ros2 action send_goal /hyfleet_compression/control_compressor \
  mserve_interfaces/action/ControlCompressor \
  "{command: 1, target: 1, mode: 1, target_pressure: 350.0}"

# SYNC both boosters
ros2 action send_goal /hyfleet_compression/control_compressor \
  mserve_interfaces/action/ControlCompressor \
  "{command: 1, target: 3, mode: 1, target_pressure: 900.0}"
```

Bringup order: `base → drivechain → low_booster → high_booster → hyfleet_compression`

---

## Dependencies

- `behaviortree_cpp` — `ros-jazzy-behaviortree-cpp` (apt)
- `behaviortree_ros2` — source build (BehaviorTree/BehaviorTree.ROS2)
- `rclcpp_action`, `rclcpp_lifecycle`
- `mserve_interfaces` — `ControlCompressor.action`, `ControlBooster.action`, `CompressorCmd.srv`
- `mserve_utils` — `get_or_declare_param` helpers

See also: [architecture.md](docs/architecture.md), [todo.md](docs/todo.md)

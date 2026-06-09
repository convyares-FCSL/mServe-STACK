# hyfleet_template

Non-buildable skeleton for HyFleet subsystem nodes. Contains a COLCON_IGNORE to prevent it from being included in workspace builds.

Use this as the starting point for any new HyFleet subsystem node that follows the lifecycle + BT pattern.

---

## Bootstrap: creating a new subsystem

### Step 1 — Copy and rename

```bash
cp -r ws/src/hyfleet_template ws/src/hyfleet_<name>
rm ws/src/hyfleet_<name>/COLCON_IGNORE
```

### Step 2 — Find and replace

Run these substitutions across the entire package (case-sensitive):

| Token                    | Replace with                        | Example                          |
|--------------------------|-------------------------------------|----------------------------------|
| `hyfleet_subsystem`      | `hyfleet_<name>`                    | `hyfleet_valve`                  |
| `SubsystemNode`          | `<Name>Node`                        | `ValveNode`                      |
| `SubsystemAction`        | `<Name>Action`                      | `ValveAction`                    |
| `ControlSubsystem`       | `Control<Name>`                     | `ControlValve`                   |
| `subsystem_node`         | `<name>_node`                       | `valve_node`                     |
| `subsystem_action`       | `<name>_action`                     | `valve_action`                   |
| `subsystem_bt_nodes`     | `<name>_bt_nodes`                   | `valve_bt_nodes`                 |
| `subsystem_params`       | `<name>_params`                     | `valve_params`                   |
| `subsystem_limits`       | `<name>_limits`                     | `valve_limits`                   |
| `hyfleet_subsystem`      | `hyfleet_<name>` (namespace)        | `hyfleet_valve`                  |
| `"subsystem_node"` (ROS node name) | `"hyfleet_<name>"` or `"<name>_node"` | `"hyfleet_valve"` |

```bash
# Quick rename — run from the new package root:
SUBSYSTEM=valve   # change to your subsystem name
NAME=Valve        # PascalCase

find . -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.xml" -o -name "*.md" \
  -o -name "CMakeLists.txt" -o -name "package.xml" \) \
  -exec sed -i \
    -e "s/hyfleet_subsystem/hyfleet_${SUBSYSTEM}/g" \
    -e "s/SubsystemNode/${NAME}Node/g" \
    -e "s/SubsystemAction/${NAME}Action/g" \
    -e "s/ControlSubsystem/Control${NAME}/g" \
    -e "s/subsystem_node/${SUBSYSTEM}_node/g" \
    -e "s/subsystem_action/${SUBSYSTEM}_action/g" \
    -e "s/subsystem_bt_nodes/${SUBSYSTEM}_bt_nodes/g" \
    -e "s/subsystem_params/${SUBSYSTEM}_params/g" \
    -e "s/subsystem_limits/${SUBSYSTEM}_limits/g" \
    -e "s/SubsystemNode/${NAME}Node/g" {} \;
```

### Step 3 — Rename files

```bash
SUBSYSTEM=valve
cd ws/src/hyfleet_${SUBSYSTEM}

git mv include/hyfleet_subsystem           include/hyfleet_${SUBSYSTEM}
git mv include/hyfleet_${SUBSYSTEM}/subsystem_node.hpp   include/hyfleet_${SUBSYSTEM}/${SUBSYSTEM}_node.hpp
git mv include/hyfleet_${SUBSYSTEM}/subsystem_limits.hpp include/hyfleet_${SUBSYSTEM}/${SUBSYSTEM}_limits.hpp
git mv src/include/subsystem_action.hpp    src/include/${SUBSYSTEM}_action.hpp
git mv src/include/subsystem_bt_nodes.hpp  src/include/${SUBSYSTEM}_bt_nodes.hpp
git mv src/subsystem_node.cpp              src/${SUBSYSTEM}_node.cpp
git mv src/subsystem_action.cpp            src/${SUBSYSTEM}_action.cpp
git mv src/subsystem_bt_nodes.cpp          src/${SUBSYSTEM}_bt_nodes.cpp
git mv src/subsystem_params.cpp            src/${SUBSYSTEM}_params.cpp
```

### Step 4 — Wire up your action interface

1. Define `Control<Name>` in `mserve_interfaces/action/` (see `ControlBooster.action` as a model).
2. Add the interface dep to `package.xml` and `CMakeLists.txt`.
3. Update `src/include/<name>_action.hpp` — the using-alias and goal log message.
4. Update `include/hyfleet_<name>/<name>_node.hpp` — replace the `ControlSubsystem` using-alias.

### Step 5 — Fill in the TODOs

Search for `// TODO:` across the package — they are ordered roughly as you'll hit them:

| File | What to fill in |
|------|----------------|
| `CMakeLists.txt` | Project name, library/executable names, install paths |
| `package.xml` | Package name and description |
| `<name>_limits.hpp` | Hardware constexpr limits (or leave empty if params cover it) |
| `<name>_node.hpp` | Member list — telemetry cache, pressure limits, etc. |
| `<name>_params.cpp` | `declare_params()` / `load_params()` — hardware indices, operational params, blackboard contracts |
| `<name>_node.cpp` | ROS node name, action server topic name, `register_bt_nodes()`, `build_bt_trees()`, `select_tree()`, feedback block, goal validation |
| `<name>_bt_nodes.hpp` | Declare BT node classes |
| `<name>_bt_nodes.cpp` | Implement BT node methods |
| `src/trees/main_tree.xml` | Replace AlwaysSuccess with real tree; add tree files for each command |

### Step 6 — Telemetry (if needed)

If this node owns a hardware telemetry subscription (like booster):

1. Create `<name>_telemetry_cache.hpp` in `include/hyfleet_<name>/` — see `hyfleet_booster` for the thread-safe cache pattern.
2. Add `rclcpp::Subscription` and cache `shared_ptr` to the node header.
3. Create the subscription and cache in `on_configure()`, place cache on blackboard.
4. Reset subscription and cache in `on_cleanup()`, before `bt_node_.reset()`.
5. BT nodes receive the cache via `blackboard_->get("telemetry_cache", cache)`.

If this node is a coordinator (like compressor) that receives feedback from child action servers, skip telemetry — write feedback keys from `onFeedback()` in the RosActionNode BT node.

### Step 7 — Build and smoke test

```bash
colcon build --packages-select hyfleet_<name>
source install/setup.bash
ros2 run hyfleet_<name> <name>_node
ros2 lifecycle set /<name>_node configure
ros2 lifecycle set /<name>_node activate
ros2 action send_goal /<name>_node/control_<name> mserve_interfaces/action/Control<Name> "{command: 1}"
```

---

## Architecture pattern

This template implements the HyFleet subsystem node pattern:

```
on_configure  → declare action server, bt_node_, blackboard, load_params, register BT nodes, build trees
on_activate   → toggle_enable(true)  — starts accepting goals
on_deactivate → toggle_enable(false) — stops accepting goals, halts active tree
on_cleanup    → destroy everything in reverse order
on_shutdown   → halt tree, abort active goal
```

```
Goal received
    → handle_goal (ACCEPT or REJECT)
    → handle_accepted → on_<name>_goal_accepted
        → validate goal fields
        → write to blackboard
        → select_tree(command)
        → set_tick_timer(true) [100 ms tick]

Tick timer fires → tick_tree_once()
    → active_tree_->tickOnce()
    → publish_feedback from blackboard keys
    → on SUCCESS/FAILURE: succeed/abort goal, halt tree, stop timer
```

Three-layer control authority:
- **ROS** (this node) — process-level command routing via BT
- **PLC** — machine-level safety interlocks (hardware)
- **Safety PLC** — hardwired human safety layer (hardware, never bypassed)

---

## Key decisions

| Decision | Rationale |
|----------|-----------|
| `LifecycleNode` | Managed startup/teardown — configure before activate, deterministic cleanup |
| `MultiThreadedExecutor` + Reentrant callback group for action server | Prevents action callbacks from blocking the BT tick timer |
| Tick timer at 100 ms | Fast enough for BT responsiveness, slow enough to avoid spin-waste |
| `bt_node_` separate from lifecycle node | RosServiceNode / RosActionNode need a plain `rclcpp::Node` for service discovery |
| Telemetry cache on blackboard | Decouples subscription ownership (node) from consumers (BT nodes); thread-safe |
| Params locked in UNCONFIGURED state | Prevents live parameter changes from invalidating loaded BT trees |
| `(void)blackboard_->get(...)` | Suppresses `[[nodiscard]]` for optional reads where 0 default is acceptable |

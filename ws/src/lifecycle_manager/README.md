# mserve_lifecycle_manager

(ROS package name is `lifecycle_manager` — the `mserve_` prefix only survives
in the C++ include path/namespace, same as `utils`/`mserve_utils`. Use
`lifecycle_manager` in `colcon build`/`ros2 run`/`find_package()`.)

BehaviorTree.CPP lifecycle manager for mServe using `behaviortree_ros2`.
Automatically configures and activates managed nodes in order, idempotently and with retries.

## What it does

- Loads `trees/bringup.xml` at startup — no C++ changes needed to add nodes
- Checks current lifecycle state before each transition — safe to re-run
- Drives `mserve_drivechain` then `mserve_base` through configure → activate
- Retries failed transitions up to 3 times before giving up
- Non-blocking 100ms timer tick alongside live ROS executor
- On SIGINT/SIGTERM, runs `trees/shutdown.xml` (deactivate/shutdown both
  nodes) while the ROS context is still valid, then exits itself — see
  `docs/lifecycle_manager/todo.md`'s "Shutdown ordering" entry (repo root
  `docs/`) for why this isn't just the default `rclcpp::on_shutdown()` hook

## Running more than one instance

Which tree files get loaded is controlled by two string params,
`bringup_tree_file`/`shutdown_tree_file` (both resolved relative to this
package's own `share/lifecycle_manager/trees/`, default `bringup.xml`/
`shutdown.xml`) — so a second instance can manage a different node (or set
of nodes) with its own tree pair, without touching the always-on one. Give
it a distinct node name too (`-r __node:=...`, or `name=` on a launch_ros
`Node` action), otherwise both instances register as `/lifecycle_manager`
and collide. `ws/src/launch/launch/mserve_slam.launch.py` does exactly this
for `slam_toolbox` — see `trees/slam_bringup.xml`/`trees/slam_shutdown.xml`
for the pattern (identical to a single-node sequence out of the main
`bringup.xml`/`shutdown.xml`, just in their own file). This is deliberately
a *second process*, not an extra sequence appended to the main tree: SLAM is
opt-in, and a node that isn't running yet would just add a ~4s dead-service
timeout to every normal startup if it lived in the tree that always runs.

## Package structure

```
include/mserve_lifecycle_manager/
  lifecycle_manager.hpp     Class declaration
src/
  lifecycle_manager.cpp     LifecycleManager, IsInState, ChangeStateNode
  main.cpp                  Entry point, signal handling
  trees/
    bringup.xml             Bringup tree — edit to change bringup order
    shutdown.xml             Shutdown tree — edit to change teardown order
    slam_bringup.xml         Bringup tree for the opt-in slam_toolbox instance
    slam_shutdown.xml        Shutdown tree for the opt-in slam_toolbox instance
    node_models.xml          Generated at startup, for Groot2's palette
```

Docs for this package live at the repo root, not alongside the source —
`docs/lifecycle_manager/lesson_plan.md` (concept notes from learning
sessions) and `docs/lifecycle_manager/todo.md` (remaining work and known
limitations).

## System setup — fresh machine

`behaviortree_ros2` is vendored (not apt-installed) at
`ws/src/third_party/BehaviorTree.ROS2/` (gitignored, `humble` branch — the
apt-packaged `behaviortree_cpp` core is new enough that this branch builds
fine against it). `btcpp_ros2_samples` inside that clone carries a
`COLCON_IGNORE` — it's not needed. Full instructions:
[`ws/src/third_party/README.md`](../third_party/README.md).

```bash
# BT.CPP core (apt)
sudo apt install ros-jazzy-behaviortree-cpp

# behaviortree_ros2 build deps (apt)
sudo apt install libboost-dev ros-jazzy-generate-parameter-library

# behaviortree_ros2 source build
cd ~/mServe-STACK/ws/src/third_party
git clone --branch humble https://github.com/BehaviorTree/BehaviorTree.ROS2.git
touch BehaviorTree.ROS2/btcpp_ros2_samples/COLCON_IGNORE

# Build in order
cd ~/mServe-STACK/ws
colcon build --packages-select btcpp_ros2_interfaces --cmake-args -DBUILD_TESTING=OFF --symlink-install
colcon build --packages-select behaviortree_ros2 --cmake-args -DBUILD_TESTING=OFF --symlink-install
colcon build --packages-select lifecycle_manager --cmake-args -DBUILD_TESTING=OFF --symlink-install
```

## Running via launch (recommended)

```bash
cd ~/mServe-STACK/ws
source install/setup.bash
ros2 launch launch mserve_min.launch.py
```

`scripts/run_stack.sh` is the normal entry point (it invokes this launch
file and picks `backend`/`uart_device` for you) — see the top-level readme.

## Running manually

```bash
source install/setup.bash

# Terminal 1
ros2 run mserve_base base_node

# Terminal 2
ros2 run mserve_drivechain drivechain_node

# Terminal 3
ros2 run lifecycle_manager lifecycle_manager
```

## How to add a managed node (XML only, no C++ needed)

```xml
<Sequence name="my_node_sequence">
    <Fallback name="config_my_node">
        <Inverter>
            <IsInState name="check_unconfigured" node_name="my_node" state="unconfigured"/>
        </Inverter>
        <RetryUntilSuccessful num_attempts="3">
            <ChangeStateNode name="configure" node_name="my_node" transition="configure"/>
        </RetryUntilSuccessful>
    </Fallback>
    <Fallback name="activate_my_node">
        <Inverter>
            <IsInState name="check_active" node_name="my_node" state="inactive"/>
        </Inverter>
        <RetryUntilSuccessful num_attempts="3">
            <ChangeStateNode name="activate" node_name="my_node" transition="activate"/>
        </RetryUntilSuccessful>
    </Fallback>
</Sequence>
```

For a shutdown-tree entry, use one of the `shutdown_*` transitions below —
`transition="shutdown"` on its own is not a recognized name and will fail
`ChangeStateNode`'s lookup.

## Lifecycle transitions

| Name                 | From         | To           |
|----------------------|--------------|--------------|
| configure            | unconfigured | inactive     |
| activate             | inactive     | active       |
| deactivate           | active       | inactive     |
| cleanup              | inactive     | unconfigured |
| shutdown_unconfigured| unconfigured | finalized    |
| shutdown_inactive    | inactive     | finalized    |
| shutdown_active      | active       | finalized    |

(names come from `mserve_utils::lifecycle::transitionIdFromName`, `ws/src/utils/include/mserve_utils/lifecycle.hpp`)

## Dependencies

- `behaviortree_cpp` — `ros-jazzy-behaviortree-cpp` (apt)
- `behaviortree_ros2` — vendored source build (`ws/src/third_party/BehaviorTree.ROS2/`, `humble` branch)
- `btcpp_ros2_interfaces` — vendored source build (same repo)
- `utils` — transition name lookup (`mserve_utils/lifecycle.hpp`)
- `lifecycle_msgs`, `ament_index_cpp`, `rclcpp`

## Learning snapshots

| Stage | Location | What it covers |
|---|---|---|
| Stage 1 | `../../learning/btcpp_stage_1/` | Raw BT.CPP — `SyncActionNode`, manual service calls |
| Stage 2 | `../../learning/btcpp_stage_2/` | `behaviortree_ros2` — `RosServiceNode`, non-blocking, file split |

# mserve_lifecycle_manager

BehaviorTree.CPP lifecycle manager for mServe using `behaviortree_ros2`.
Automatically configures and activates managed nodes in order, idempotently and with retries.

## What it does

- Loads `trees/bringup.xml` at startup — no C++ changes needed to add nodes
- Checks current lifecycle state before each transition — safe to re-run
- Drives `mserve_base` and `mserve_drivechain` through configure → activate
- Retries failed transitions up to 3 times before giving up
- Non-blocking 100ms timer tick alongside live ROS executor

## Package structure

```
include/mserve_lifecycle_manager/
  lifecycle_manager.hpp     Class declaration
src/
  lifecycle_manager.cpp     LifecycleManager, IsInState, ChangeStateNode
  main.cpp                  Entry point
  trees/
    bringup.xml             Tree definition — edit to change bringup order
docs/
    lesson_plan.md              Concept notes from learning sessions
    todo.md                     Remaining work and phase roadmap
```

## System setup — fresh machine

```bash
# BT.CPP core
sudo apt install ros-jazzy-behaviortree-cpp

# behaviortree_ros2 build dep
sudo apt install ros-jazzy-generate-parameter-library

# behaviortree_ros2 source build
cd ~/mServe-STACK/ws/src
git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git

# Build in order
cd ~/mServe-STACK/ws
colcon build --packages-select btcpp_ros2_interfaces
colcon build --packages-select behaviortree_ros2
colcon build --packages-select mserve_lifecycle_manager --symlink-install
```

## Running via launch (recommended)

```bash
cd ~/mServe-STACK/ws
source install/setup.bash
ros2 launch mserve_launch mserve_min.launch.py
```

## Running manually

```bash
source install/setup.bash

# Terminal 1
ros2 run mserve_base base_node

# Terminal 2
ros2 run mserve_drivechain drivechain_node

# Terminal 3
ros2 run mserve_lifecycle_manager lifecycle_manager
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
        <IsInState name="check_active" node_name="my_node" state="active"/>
        <RetryUntilSuccessful num_attempts="3">
            <ChangeStateNode name="activate" node_name="my_node" transition="activate"/>
        </RetryUntilSuccessful>
    </Fallback>
</Sequence>
```

## Lifecycle transitions

| Name       | From         | To           |
|------------|-------------|-------------|
| configure  | unconfigured | inactive    |
| activate   | inactive     | active      |
| deactivate | active       | inactive    |
| cleanup    | inactive     | unconfigured|
| shutdown   | any          | finalized   |

## Dependencies

- `behaviortree_cpp` — `ros-jazzy-behaviortree-cpp` (apt)
- `behaviortree_ros2` — source build (BehaviorTree/BehaviorTree.ROS2)
- `btcpp_ros2_interfaces` — source build (same repo)
- `mserve_utils` — transition name lookup (`lifecycle.hpp`)
- `lifecycle_msgs`, `ament_index_cpp`, `rclcpp`

## Learning snapshots

| Stage | Location | What it covers |
|---|---|---|
| Stage 1 | `../../learning/btcpp_stage_1/` | Raw BT.CPP — `SyncActionNode`, manual service calls |
| Stage 2 | `../../learning/btcpp_stage_2/` | `behaviortree_ros2` — `RosServiceNode`, non-blocking, file split |

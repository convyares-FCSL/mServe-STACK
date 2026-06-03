# mserve_bringup_bt

BehaviorTree.CPP-based lifecycle manager for mServe. Replaces manual web UI lifecycle control with an automatic behaviour tree that configures and activates managed nodes in order.

## What it does

- Loads a behaviour tree from `trees/bringup.xml` at startup
- For each managed node in the tree, creates a `ChangeState` service client at tree load time
- Ticks the tree, driving each lifecycle node through configure → activate in sequence
- Returns `FAILURE` if any transition is rejected or times out

## Package structure

```
src/
  main.cpp              ROS 2 node (BaseNode) + BT nodes (ChangeStateNode)
  trees/
    bringup.xml         Tree definition — edit this to change bringup order
```

## Running natively (no Docker)

```bash
cd ~/ai-workspace/projects/mServe-STACK/ws
source install/setup.bash

# Terminal 1 — start managed node(s)
ros2 run mserve_base base_node

# Terminal 2 — run the BT lifecycle manager
ros2 run mserve_bringup_bt bringup_bt
```

## How to add a node to bringup

1. Add a `<ChangeStateNode>` entry in `trees/bringup.xml`:
   ```xml
   <ChangeStateNode name="configure_drivechain" node_name="mserve_drivechain" transition="1"/>
   ```
2. Rebuild — no C++ changes needed.

## Transition IDs

| ID | Transition  |
|----|-------------|
| 1  | configure   |
| 3  | activate    |
| 4  | deactivate  |
| 6  | cleanup     |

## Dependencies

- `behaviortree_cpp` (4.9.0, installed via `ros-jazzy-behaviortree-cpp`)
- `lifecycle_msgs`
- `ament_index_cpp`
- `rclcpp`

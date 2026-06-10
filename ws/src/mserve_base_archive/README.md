# mserve_base

Command arbiter and safety gate for the mServe robot.

## Responsibilities

- Subscribe to `/cmd_vel` (from Nav2, joystick, or any command source)
- Clamp linear and angular velocity to robot-level safety limits
- Publish the clamped command on `/mserve/cmd_vel_safe` for `mserve_drivechain`
- Accept hot parameter updates for speed limits without reconfiguring

## What this node does NOT own

- Wheel geometry (`wheel_separation`, `wheel_radius`) — that belongs to `mserve_drivechain`
- Kinematics (Twist → wheel speeds) — same
- Motor protocol or hardware communication — same

When Nav2 and a joystick are added, this node will also arbitrate between command sources. For now it is a single-source safety clamp.

## Topic boundary

```
/cmd_vel  →  BaseNode (clamp)  →  /mserve/cmd_vel_safe  →  mserve_drivechain
```

## Parameters

All parameters live under `mserve_base:` in `mserve_interfaces/config/mserve_params.yaml`.

| Parameter | Default | Range | Hot-swap |
|-----------|---------|-------|----------|
| `limits.max_linear_speed` | `0.8` m/s | 0.01 – 10.0 | ✅ |
| `limits.max_angular_speed` | `1.2` rad/s | 0.01 – 10.0 | ✅ |
| `topic_names.cmd_vel` | `/cmd_vel` | — | ❌ reconfigure |
| `topic_names.cmd_vel_safe` | `/mserve/cmd_vel_safe` | — | ❌ reconfigure |
| `qos.commands.*` | reliable, volatile, depth=1 | — | ❌ reconfigure |

Hot-swap means the value takes effect immediately via `ros2 param set` without restarting or reconfiguring the node. ROS rejects values outside the declared range before the callback is called.

## Runtime parameter update

```bash
ros2 param set /mserve_base limits.max_linear_speed 0.5
ros2 param set /mserve_base limits.max_angular_speed 0.8
```

Describe a parameter to see its bounds:

```bash
ros2 param describe /mserve_base limits.max_linear_speed
```

## Build

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select mserve_base --symlink-install
```

## Run standalone

```bash
source install/setup.bash
ros2 run mserve_base base_node \
  --ros-args --params-file install/mserve_interfaces/share/mserve_interfaces/config/mserve_params.yaml
```

## Launch (full system)

```bash
cd /home/ecm/mServe-STACK
scripts/05_utils/docker_launch_mserve.sh
```

## Dependencies

- `geometry_msgs` — Twist in/out
- `rclcpp` / `rclcpp_lifecycle` — lifecycle node
- `mserve_utils` — `get_or_declare_param`, `bounded_double`, `mserve_qos`, `mserve_topics`

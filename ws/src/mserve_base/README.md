# mserve_base

Command arbiter and safety gate for mServe. Subscribes to `/cmd_vel`, applies
robot-level speed limits and a dead-man-switch timeout, converts the result to
per-wheel motor commands via differential-drive kinematics, and forwards them
to `mserve_drivechain`'s `~/drive` service.

`mserve_drivechain` stays a pure motor driver (no kinematics); `mserve_base`
owns the robot geometry and the `/cmd_vel` → wheel-RPM conversion.

## Architecture

```
BaseNode (lifecycle)
│
├── Subscriber  /cmd_vel                          (geometry_msgs/Twist)
│     └── CmdVelStore::update() — thread-safe; no BT
├── Subscriber  /mserve_drivechain/motor_feedback  (DriveMotorFeedback)
│     └── stored directly on the blackboard for UpdateOdometry
│
├── Timer  (feedback_rate Hz, default 10 Hz)
│     └── drive_tree.xml:
│           ApplyCmdVelSafety  — read CmdVelStore; zero if stale (cmd_vel_timeout_ms);
│                                 clamp to limits.*; publish ~/cmd_vel_safe
│           ComputeKinematics  — safe Twist + geometry.* + motor_ids.* → wheel RPM
│           CallDriveService   — async call to mserve_drivechain ~/drive (best-effort)
│           PublishBaseStatus  — publish ~/base_status
│           UpdateOdometry     — integrate odom pose + wheel angles from wheel *velocity*
│           PublishOdometry    — publish /odom, broadcast odom->base_link TF, publish /joint_states
│
├── Publisher  /mserve/cmd_vel_safe   (geometry_msgs/Twist)
├── Publisher  ~/base_status          (DriveStatus)
├── Publisher  /odom                  (nav_msgs/Odometry)
├── Publisher  /joint_states           (sensor_msgs/JointState)
├── Broadcaster odom -> base_link TF  (tf2_ros::TransformBroadcaster)
└── Client     /mserve_drivechain/drive (interfaces/srv/Drive)
```

### Odometry — why it integrates velocity, not position

`mserve_drivechain`'s DDSM115 protocol only reports wheel position in
position-control mode; in the speed-loop mode `mserve_base` actually drives
in, `position_rad` is always `0` (`drivechain_uart.cpp`'s
`parse_json_feedback`: *"not returned in speed-loop feedback"*). So
`UpdateOdometry` integrates `velocity_rads` over time instead of differencing
an absolute position — the one feedback signal that's reliable in
speed-loop mode. It also self-integrates `left_wheel_angle`/`right_wheel_angle`
the same way, purely for `/joint_states` (so RViz can resolve the wheel TF
frames), since `mserve_drivechain` can't provide real wheel angle either.
`nav_msgs/Odometry`'s covariance fields are left at zero (no real uncertainty
estimate yet) — fine for RViz/basic use, but tune before feeding this into
Nav2/AMCL.

Command flow: `/cmd_vel` (Nav2, joystick, or the web UI) is cached in
`CmdVelStore`. The drive timer reads the cache on every tick. If no message
arrives within `drive.cmd_vel_timeout_ms`, the output Twist is zeroed
(dead-man switch) — same effect as the old standalone `mserve_base` clamp
node, but now followed by kinematics + a `~/drive` call instead of just a
republish.

The **blackboard** is the single source of truth shared between the node and
all BT nodes:

| Key | Type | Written by | Read by |
|---|---|---|---|
| `cmd_vel_store` | `CmdVelStore*` | on_configure | ApplyCmdVelSafety |
| `drive_client` | `rclcpp::Client<Drive>::SharedPtr` | on_configure | CallDriveService |
| `max_linear_speed` / `max_angular_speed` | double | load_params, on_parameters | ApplyCmdVelSafety |
| `wheel_separation` / `wheel_radius` / `gear_ratio` | double | load_params | ComputeKinematics |
| `left_motor_id` / `right_motor_id` | int | load_params | ComputeKinematics, send_zero_drive |
| `cmd_vel_timeout_ms` | int | load_params, on_parameters | ApplyCmdVelSafety |
| `feedback_rate` | double | load_params, on_parameters | on_activate, on_parameters |
| `safe_twist` | `geometry_msgs::msg::Twist` | ApplyCmdVelSafety | ComputeKinematics |
| `wheel_commands` | `vector<MotorCommand>` ([left, right]) | ComputeKinematics | CallDriveService |
| `drivechain_reachable` | bool | CallDriveService | PublishBaseStatus |
| `motor_feedback` | `DriveMotorFeedback` | on_motor_feedback | UpdateOdometry, PublishOdometry |
| `odom_x` / `odom_y` / `odom_theta` | double | UpdateOdometry | UpdateOdometry (next tick), PublishOdometry |
| `left_wheel_angle` / `right_wheel_angle` | double | UpdateOdometry | PublishOdometry (`/joint_states`) |
| `linear_velocity` / `angular_velocity` | double | UpdateOdometry | PublishOdometry |
| `publish_cmd_vel_safe` | `std::function` | on_configure | ApplyCmdVelSafety |
| `publish_base_status` | `std::function` | on_configure | PublishBaseStatus |
| `publish_odom` / `publish_odom_tf` / `publish_joint_states` | `std::function` | on_configure | PublishOdometry |

### Key source files

| File | Purpose |
|---|---|
| `include/mserve_base/base_node.hpp` | Node public API |
| `src/base_node.cpp` | Lifecycle + BT runner + cmd_vel/motor_feedback subscriptions |
| `src/base_params.cpp` | Parameter declaration, loading, hot-change |
| `src/base_bt_nodes.cpp` | All BT node implementations |
| `src/include/base_types.hpp` | `WheelDescriptor`, `CmdVelStore` |
| `src/trees/drive_tree.xml` | BT tree definition |

## Parameters

| Parameter | Default | Notes |
|---|---|---|
| `limits.max_linear_speed` | `0.8` m/s | Robot-level safety cap. Hot-changeable. |
| `limits.max_angular_speed` | `1.2` rad/s | Robot-level safety cap. Hot-changeable. |
| `geometry.wheel_separation` | `0.35` m | Track width between wheel centers |
| `geometry.wheel_radius` | `0.08` m | Wheel radius |
| `geometry.gear_ratio` | `1.0` | Motor revolutions per wheel revolution |
| `motor_ids.left` | `2` | `mserve_drivechain` motor ID for the left wheel |
| `motor_ids.right` | `1` | `mserve_drivechain` motor ID for the right wheel |
| `drive.cmd_vel_timeout_ms` | `500` | Send a zero drive command if no `/cmd_vel` within this window |
| `feedback_rate` | `10.0` | Drive loop tick rate (Hz). Hot-changeable. |

`geometry.*` and `motor_ids.*` can only be changed in `UNCONFIGURED` state —
the kinematics math reads them once at configure time.
`limits.*`, `drive.cmd_vel_timeout_ms`, and `feedback_rate` are
hot-changeable at runtime.

## Build

```bash
cd ~/mServe-STACK/ws
colcon build --packages-select interfaces utils mserve_drivechain mserve_base
source install/setup.bash
```

## Run (manual / CLI)

**Terminal 1 — drivechain (target of `~/drive` calls):**
```bash
source install/setup.bash
ros2 run mserve_drivechain drivechain_node
ros2 lifecycle set /mserve_drivechain configure
ros2 lifecycle set /mserve_drivechain activate
ros2 service call /mserve_drivechain/connect std_srvs/srv/Trigger
```

**Terminal 2 — base:**
```bash
source install/setup.bash
ros2 run mserve_base base_node
ros2 lifecycle set /mserve_base configure
ros2 lifecycle set /mserve_base activate
```

**Terminal 3 — drive via /cmd_vel:**
```bash
source install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10
```

### Real-time monitoring

```bash
source install/setup.bash

# Clamped/dead-man-gated Twist actually sent to kinematics
ros2 topic echo /mserve/cmd_vel_safe

# Bridge status: "bridging" / "drivechain_unreachable"
ros2 topic echo /mserve_base/base_status

# Odometry pose/twist
ros2 topic echo /odom

# Wheel joint angles (for RViz TF resolution)
ros2 topic echo /joint_states

# odom -> base_link transform
ros2 run tf2_ros tf2_echo odom base_link
```

## BT tree

| Tree | File | Triggered by |
|---|---|---|
| Drive | `drive_tree.xml` | Timer tick (every 1/feedback_rate seconds) |

`CallDriveService` is best-effort: if `mserve_drivechain`'s `~/drive` service
isn't available yet, the tick still returns SUCCESS and `~/base_status`
reports `drivechain_unreachable`. `UpdateOdometry` is the same way — if
`motor_feedback` hasn't arrived yet, or doesn't cover both configured wheel
IDs, it returns SUCCESS without updating the pose.

# Milestones

> **As implemented:** package names below have moved on — see `docs/packages.md`
> for the current name (`interfaces`/`utils`/`launch`, not `mserve_interfaces`/
> `mserve_utils`/`mserve_bringup`; no separate `mserve_esp32`). Acceptance-check
> commands are kept close to their original phrasing for history; use
> `docs/packages.md`/package READMEs for the commands that actually work today.

## ✅ Milestone 0: Workspace Hygiene

Create:

```text
ws/src/
scripts/01_setup/
scripts/02_bootstrap/
scripts/03_packages/
scripts/04_tests/
scripts/05_utils/
```

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK
./scripts/05_utils/env_setup.sh
./scripts/05_utils/clean_all.sh
```

## ✅ Milestone 1: Interfaces and Central Config

Create `interfaces` (package name — see note above).

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select interfaces --symlink-install
source install/setup.bash
ros2 interface show interfaces/msg/DriveStatus
ros2 interface show interfaces/msg/MotorCommand
```

## ✅ Milestone 2: C++ Utils

Create `utils`.

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select interfaces utils --symlink-install
colcon test --packages-select utils --event-handlers console_direct+
colcon test-result --verbose
```

## ✅ Milestone 3: Minimal Base and Drivechain

Create `mserve_base` and `mserve_drivechain` (the latter absorbed the
planned `mserve_esp32` boundary — see `docs/packages.md`).

First behavior:

- `mserve_base` starts as a lifecycle node.
- `mserve_base` clamps `/cmd_vel` and publishes the safe Twist.
- `mserve_drivechain` starts as a lifecycle node, `sim` backend by default.
- `mserve_drivechain` runs diff-drive kinematics and publishes fake
  feedback/status in `sim` mode; talks real JSON/UART to the ESP32 in
  `hardware` mode.

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select mserve_base mserve_drivechain --symlink-install
source install/setup.bash
ros2 run mserve_base base_node
ros2 run mserve_drivechain drivechain_node --ros-args -p drive.backend:=sim
```

## ✅ Milestone 4: Minimal Bringup

Create `launch` (planned as `mserve_bringup`) and `mserve_min.launch.py`.

**As implemented**, this milestone grew a dependency the original plan
didn't have: lifecycle transitions are driven by `lifecycle_manager` (a
BehaviorTree.CPP node, not in the original plan — see `docs/packages.md`),
launched alongside the two nodes rather than done by hand.

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select interfaces utils mserve_drivechain mserve_base lifecycle_manager launch \
  btcpp_ros2_interfaces behaviortree_ros2 --cmake-args -DBUILD_TESTING=OFF --symlink-install
source install/setup.bash
ros2 launch launch mserve_min.launch.py backend:=sim
ros2 lifecycle get /mserve_base
ros2 lifecycle get /mserve_drivechain
```

## Milestone 5: Unit Tests — partial

What exists today: `utils/test/test_config.cpp`,
`mserve_drivechain/test/test_packet_codec.cpp`. `mserve_base/test/` exists
but is empty. Still missing from the original plan: diff-drive math tests,
command-clamping tests, timeout/fail-safe logic tests, QoS/validation tests.

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon test --packages-select utils mserve_base mserve_drivechain --event-handlers console_direct+
colcon test-result --verbose
```

## Milestone 6: Robot Description — partial

`mserve_description`'s URDF already covers `base_link`, wheels
(`mserve_core.xacro`), a depth camera (`mserve_depth_camera.xacro`, active)
and a plain camera (`mserve_camera.xacro`, currently commented out), and a
lidar (`mserve_lidar.xacro`) — but as of 2026-07-12 no physical camera or
lidar is actually connected to the Pi (see `docs/continue.md`), so these
are mounting-point/sim definitions only, not backed by a real driver node
yet. No display frame or future arm mount frame exist yet.

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select mserve_description --symlink-install
source install/setup.bash
ros2 launch launch mserve_min.launch.py backend:=sim
```

## Milestone 7: Launch Tests — not done

`launch/test/test_launch_descriptions.py` exists but is a stub (`assert
isinstance(LaunchDescription(), LaunchDescription)`) — none of the acceptance
criteria below are actually exercised yet.

Add `launch_testing` integration tests:

- Minimal launch starts.
- Lifecycle nodes are visible.
- Configure/activate/deactivate works.
- Expected topics exist.

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon test --packages-select launch --event-handlers console_direct+
```

## Milestone 8: Gazebo Simulation — partial

No standalone `mserve_sim` package was created; `mserve_description`
depends on `ros_gz_bridge`/`ros_gz_sim` and its xacros carry Gazebo
`<sensor>` tags directly (see `docs/packages.md`), but there's no launch
path yet that actually starts Gazebo and spawns the robot — Gazebo + RViz
run on the NVIDIA Thor, not the Pi (see `docs/simulation_hil.md`).

Acceptance checks (not yet passing — no `mserve_sim` package or sim launch
file exists to run them):

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select mserve_description --symlink-install
source install/setup.bash
# no ros2 launch ... mserve_sim.launch.py yet
```

Expected:

- Gazebo starts.
- mServe spawns.
- `/clock`, `/scan`, camera topic, and motor command/feedback path exist.

## Milestone 9: Nav2 Simulation — not started

No `mserve_navigation` package, no Nav2 dependency anywhere in `ws/src/` yet.

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select mserve_navigation launch --symlink-install
source install/setup.bash
ros2 launch launch mserve_nav.launch.py
```

Expected:

- Nav2 lifecycle nodes start.
- RViz can send a navigation goal.
- Robot moves in Gazebo through the same command boundary.

## Milestone 10: Composition and Fault Lines

Add composable component registration once standalone nodes are stable.

Topology target:

- Control plane: base safety/status and display.
- Motor comms plane: ESP32 node isolated enough to fail safely.
- Sensor plane: camera/lidar isolated when data rates become heavy.
- Nav2 plane: separate stack.
- Future AI/manipulation plane: isolated worker packages.

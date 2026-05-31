# Milestones

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

Create `mserve_interfaces`.

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select mserve_interfaces --symlink-install
source install/setup.bash
ros2 interface show mserve_interfaces/msg/DriveStatus
ros2 interface show mserve_interfaces/msg/WheelCommand
```

## ✅ Milestone 2: C++ Utils

Create `mserve_utils`.

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select mserve_interfaces mserve_utils --symlink-install
colcon test --packages-select mserve_utils --event-handlers console_direct+
colcon test-result --verbose
```

## ✅ Milestone 3: Minimal Base and ESP32 Stubs

Create `mserve_base` and `mserve_esp32`.

First behavior:

- `mserve_base` starts as a lifecycle node.
- `mserve_base` clamps `/cmd_vel` and publishes wheel commands.
- `mserve_esp32` starts as a lifecycle node in dry-run mode.
- `mserve_esp32` subscribes to wheel commands and publishes fake feedback/status.

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select mserve_base mserve_esp32 --symlink-install
source install/setup.bash
ros2 run mserve_base mserve_base_node
ros2 run mserve_esp32 mserve_esp32_node
```

## ✅ Milestone 4: Minimal Bringup

Create `mserve_bringup` and `mserve_min.launch.py`.

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --symlink-install
source install/setup.bash
ros2 launch mserve_bringup mserve_min.launch.py
ros2 lifecycle get /mserve_base
ros2 lifecycle get /mserve_esp32
```

## Milestone 5: Unit Tests

Add GTest coverage for:

- Diff-drive math.
- Command clamping.
- ESP32 packet encoding/decoding.
- Timeout/fail-safe logic.
- QoS/config validation.

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon test --packages-select mserve_utils mserve_base mserve_esp32 --event-handlers console_direct+
colcon test-result --verbose
```

## Milestone 6: Robot Description

Add a simple robot description with:

- `base_link`
- left/right wheel links
- lidar frame
- camera frame
- display frame
- future arm mount frame

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select mserve_description --symlink-install
source install/setup.bash
ros2 launch mserve_bringup mserve_min.launch.py
```

## Milestone 7: Launch Tests

Add `launch_testing` integration tests:

- Minimal launch starts.
- Lifecycle nodes are visible.
- Configure/activate/deactivate works.
- Expected topics exist.

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon test --packages-select mserve_bringup --event-handlers console_direct+
```

## Milestone 8: Gazebo Simulation

Create `mserve_sim`.

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select mserve_description mserve_sim mserve_bringup --symlink-install
source install/setup.bash
ros2 launch mserve_bringup mserve_sim.launch.py
```

Expected:

- Gazebo starts.
- mServe spawns.
- `/clock`, `/scan`, camera topic, and motor command/feedback path exist.

## Milestone 9: Nav2 Simulation

Create `mserve_navigation`.

Acceptance checks:

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select mserve_navigation mserve_bringup --symlink-install
source install/setup.bash
ros2 launch mserve_bringup mserve_nav.launch.py
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

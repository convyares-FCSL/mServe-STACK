# Package Plan

## Target Workspace

> **As implemented:** folder names below are current (`ws/src/` really looks
> like this). The package *names* differ from the plan in a few spots —
> `interfaces`/`utils`/`launch` dropped the `mserve_` prefix at some point
> (their C++ include paths/namespaces kept it, e.g. `mserve_utils::`), and
> `mserve_esp32`/`mserve_bringup` were never separate packages (see their
> sections below). `lifecycle_manager` wasn't in the original plan at all —
> added later to drive lifecycle transitions via BehaviorTree.CPP. Everything
> from `mserve_lidar/` down is still just planned, not yet created.

```text
mServe-STACK/
├── docs/
├── scripts/
└── ws/
    └── src/
        ├── interfaces/
        ├── utils/
        ├── mserve_description/
        ├── launch/
        ├── mserve_base/
        ├── mserve_drivechain/       (folds in the planned mserve_esp32 boundary)
        ├── lifecycle_manager/       (not in the original plan — see its own section)
        ├── mserve_lidar/            planned
        ├── mserve_camera/           planned
        ├── mserve_display/         planned
        ├── mserve_sim/              planned
        ├── mserve_navigation/       planned
        ├── mserve_ai/               planned
        └── mserve_manipulation/     planned
```

## `interfaces` (planned as `mserve_interfaces`)

Shared interfaces and central config.

```text
interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── DriveStatus.msg
│   ├── DisplayStatus.msg
│   ├── Esp32Status.msg
│   ├── MotorCommand.msg
│   ├── MotorState.msg
│   └── DriveMotorFeedback.msg
├── srv/
│   ├── Drive.srv
│   ├── SetDisplayMode.srv
│   └── SetMotorId.srv
├── action/
│   └── Dock.action
└── config/
    ├── mserve_params.yaml   (single shared params file — see mserve_base/mserve_drivechain READMEs)
    ├── qos.yaml
    ├── robot.yaml
    └── topics.yaml
```

This mirrors the lesson `lesson_interfaces` package: shared schemas and shared YAML first.
The original plan's `WheelCommand`/`WheelFeedback` became `MotorCommand`/`MotorState`/
`DriveMotorFeedback` (per-motor, not per-wheel, once the drivechain-v2 protocol landed);
`services.yaml`/`frames.yaml`/`hardware.yaml`/`sim.yaml`/`nav2.yaml` were never split out —
config still fits in fewer files than planned.

## `utils` (planned as `mserve_utils`)

C++ helpers shared by nodes. Package name is `utils`; the C++ include path/namespace
kept the `mserve_utils` prefix (`#include <mserve_utils/...>`).

```text
utils/
├── CMakeLists.txt
├── package.xml
├── include/mserve_utils/
│   ├── config.hpp
│   ├── lifecycle.hpp
│   ├── param_guard.hpp
│   ├── qos.hpp
│   ├── topics.hpp
│   └── utils.hpp
└── test/
    └── test_config.cpp
```

Responsibilities:

- Read declared parameters safely.
- Build QoS profiles from YAML.
- Validate robot constants, frame names, timeouts, and device config.
- Keep helpers thin and predictable, like `utils_cpp`.
- `lifecycle.hpp` also backs `lifecycle_manager`'s transition-name lookup
  (`configure`/`activate`/`deactivate`/`cleanup`/`shutdown_unconfigured`/
  `shutdown_inactive`/`shutdown_active` — not a plain `shutdown` transition).

`test_qos.cpp`/`test_validation.cpp` from the original plan were never added;
only `test_config.cpp` exists today.

## `mserve_base`

C++ lifecycle package for robot-level command arbitration and safety clamping.

```text
mserve_base/
├── CMakeLists.txt
├── package.xml
├── include/mserve_base/
│   ├── base_limits.hpp
│   └── base_node.hpp
├── src/
│   ├── base_node.cpp
│   ├── base_params.cpp
│   ├── base_bt_nodes.cpp
│   ├── main.cpp
│   └── trees/
└── test/   (empty — no unit tests added yet, see docs/milestones.md Milestone 5)
```

Responsibilities (**as implemented — this changed from the original plan**:
`mserve_base` does *not* own diff-drive kinematics or wheel commands; that
moved to `mserve_drivechain` so swapping drivetrain hardware only touches
one package):

- Subscribe to `/cmd_vel`.
- Clamp to speed limits (`base_limits.hpp`).
- Publish the clamped, safe Twist on `/mserve/cmd_vel_safe`.
- Own future command arbitration (Nav2 vs. joystick vs. e-stop) — see
  `docs/architecture.md`'s Control Philosophy.

## `mserve_esp32` (as implemented: folded into `mserve_drivechain`, not a separate package)

> This was planned as its own package; in practice the ESP32 boundary was
> implemented inside `mserve_drivechain` instead (JSON-over-UART client in
> `drivechain_uart.cpp`), with the ESP32's own firmware living under
> `mserve_drivechain/drive_firmware/`. Section kept for the design intent,
> which is still accurate — just not the package split.

C++ lifecycle package for ESP32 motor-controller communication.

**As implemented**, this lives inside `mserve_drivechain` (package name kept,
not renamed) rather than a separate package — real file tree:

```text
mserve_drivechain/
├── CMakeLists.txt
├── package.xml
├── include/mserve_drivechain/
│   ├── drivechain_limits.hpp
│   └── drivechain_node.hpp
├── src/
│   ├── drivechain_node.cpp        Lifecycle node
│   ├── drivechain_params.cpp
│   ├── drivechain_uart.cpp         JSON-over-UART client to the ESP32
│   ├── drivechain_bt_nodes.cpp
│   ├── main.cpp
│   └── include/                   drivechain_types.hpp, drivechain_uart.hpp, drivechain_bt_nodes.hpp
├── drive_firmware/                 ESP32 firmware (not a ROS package — flashed onto the board)
└── test/
    └── test_packet_codec.cpp
```

Responsibilities (matches the original plan, just inside `mserve_drivechain`):

- Own diff-drive kinematics (moved here from `mserve_base` — see that
  section above) and the ESP32 serial/transport setup.
- Encode/decode the JSON-over-UART protocol; the ESP32 itself speaks the
  raw DDSM115 binary protocol to the motors.
- Publish `DriveMotorFeedback`/`MotorState`, drive status.
- Fail safe (zero motors) on comms timeout — see `drivechain_limits.hpp`.

`test_timeout_logic.cpp` from the original plan was never added; only
`test_packet_codec.cpp` exists today.

## `mserve_description`

Robot model for TF, RViz, Gazebo, and Nav2.

**As implemented**, already split past the "start simple" single-file stage
the plan describes — current tree:

```text
mserve_description/
├── CMakeLists.txt
├── package.xml
└── urdf/
    ├── mserve.urdf.xacro       top-level, includes the rest
    ├── mserve_core.xacro       base_link, wheels
    ├── mserve_camera.xacro     Pi Camera Module 3 — currently commented out
    ├── mserve_depth_camera.xacro  RGBD depth camera — currently the active camera include
    ├── mserve_lidar.xacro      RPLIDAR C1
    ├── mserve_gazebo.xacro     Gazebo-specific sim tuning (ros_gz_bridge/ros_gz_sim deps)
    └── inertial_macros.xacro
```

No physical camera or lidar is connected yet (checked 2026-07-12 — see
`docs/session.md`/`docs/continue.md`); the xacros above describe intended
mounting/sensor parameters for when hardware is picked, and the Gazebo
`<sensor>` tags are simulation-only, not tied to a real driver node yet.
No `rviz/` config or `test_urdf_load.py` exist yet.

## `launch` (planned as `mserve_bringup`)

Top-level launch package.

```text
launch/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── mserve_min.launch.py    starts mserve_drivechain + mserve_base + lifecycle_manager
└── test/
    └── test_launch_descriptions.py   (stub — asserts LaunchDescription() constructs, not a real launch_testing integration test; see Milestone 7)
```

Only `mserve_min.launch.py` exists — the planned `mserve_sim`/`mserve_nav`/
`mserve_hardware`/`containers` launch files haven't been created; sim vs.
hardware backend is currently a launch *argument* (`backend:=sim|hardware`)
on the one launch file rather than separate files. Launch should make
topology obvious, following Lesson 10.

## `lifecycle_manager`

Not in the original plan — added later. BehaviorTree.CPP node (via the
vendored `behaviortree_ros2`) that drives `mserve_drivechain` then
`mserve_base` through configure → activate on bringup, and a shutdown tree
on SIGINT/SIGTERM, instead of each launch file/node managing lifecycle
transitions by hand. See `ws/src/lifecycle_manager/README.md` for the full
package layout, the transition-name table, and how to add a new managed
node to the tree (XML only, no C++ changes needed).

## Later Packages

`mserve_lidar`:

- Wrap or adapt the chosen lidar driver.
- Keep filtering/health logic testable.

`mserve_camera`:

- Wrap or adapt camera source.
- Reserve image topics for future AI.

`mserve_display`:

- Handle robot display/status output.
- Expose `SetDisplayMode`.

`mserve_sim`:

- Start Gazebo.
- Spawn robot.
- Bridge clock, scan, camera, command, and feedback topics.

`mserve_navigation`:

- Own Nav2 config, maps, behavior tree, and launch wrappers.

`mserve_ai`:

- Future perception and task planning.

`mserve_manipulation`:

- Future arm package, likely with MoveIt 2 later.

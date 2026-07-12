# Package Plan

## Target Workspace

```text
mServe-STACK/
├── docs/
├── scripts/
└── ws/
    └── src/
        ├── mserve_interfaces/
        ├── mserve_utils/
        ├── mserve_description/
        ├── mserve_bringup/
        ├── mserve_base/
        ├── mserve_esp32/
        ├── mserve_lidar/
        ├── mserve_camera/
        ├── mserve_display/
        ├── mserve_sim/
        ├── mserve_navigation/
        ├── mserve_ai/
        └── mserve_manipulation/
```

## `mserve_interfaces`

Shared interfaces and central config.

```text
mserve_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── DriveStatus.msg
│   ├── DisplayStatus.msg
│   ├── Esp32Status.msg
│   ├── WheelCommand.msg
│   └── WheelFeedback.msg
├── srv/
│   └── SetDisplayMode.srv
├── action/
│   └── Dock.action
└── config/
    ├── topics.yaml
    ├── qos.yaml
    ├── services.yaml
    ├── frames.yaml
    ├── robot.yaml
    ├── hardware.yaml
    ├── sim.yaml
    └── nav2.yaml
```

This mirrors the lesson `lesson_interfaces` package: shared schemas and shared YAML first.

## `mserve_utils`

C++ helpers shared by nodes.

```text
mserve_utils/
├── CMakeLists.txt
├── package.xml
├── include/mserve_utils/
│   ├── config.hpp
│   ├── lifecycle.hpp
│   ├── qos.hpp
│   ├── topics.hpp
│   └── validation.hpp
└── test/
    ├── test_config.cpp
    ├── test_qos.cpp
    └── test_validation.cpp
```

Responsibilities:

- Read declared parameters safely.
- Build QoS profiles from YAML.
- Validate robot constants, frame names, timeouts, and device config.
- Keep helpers thin and predictable, like `utils_cpp`.

## `mserve_base`

C++ lifecycle package for robot-level drive control.

```text
mserve_base/
├── CMakeLists.txt
├── package.xml
├── include/mserve_base/
│   ├── diff_drive_kinematics.hpp
│   ├── drive_limits.hpp
│   └── drive_node.hpp
├── src/
│   ├── drive_node.cpp
│   └── drive_node_main.cpp
└── test/
    ├── test_diff_drive_kinematics.cpp
    └── test_drive_limits.cpp
```

Responsibilities:

- Subscribe to `/cmd_vel`.
- Clamp commands.
- Convert body velocity to wheel commands.
- Publish `WheelCommand`.
- Consume `WheelFeedback`.
- Publish `DriveStatus`.

## `mserve_esp32` (as implemented: folded into `mserve_drivechain`, not a separate package)

> This was planned as its own package; in practice the ESP32 boundary was
> implemented inside `mserve_drivechain` instead (JSON-over-UART client in
> `drivechain_uart.cpp`), with the ESP32's own firmware living under
> `mserve_drivechain/drive_firmware/`. Section kept for the design intent,
> which is still accurate — just not the package split.

C++ lifecycle package for ESP32 motor-controller communication.

```text
mserve_esp32/
├── CMakeLists.txt
├── package.xml
├── include/mserve_esp32/
│   ├── esp32_node.hpp
│   ├── packet_codec.hpp
│   ├── serial_transport.hpp
│   └── transport.hpp
├── src/
│   ├── esp32_node.cpp
│   ├── esp32_node_main.cpp
│   ├── packet_codec.cpp
│   └── serial_transport.cpp
└── test/
    ├── test_packet_codec.cpp
    └── test_timeout_logic.cpp
```

Responsibilities:

- Own ESP32 serial/transport setup.
- Encode wheel commands.
- Decode wheel feedback and ESP32 status.
- Publish comms health.
- Fail safe on timeout.

The first version can use a fake transport or dry-run mode so the package builds before hardware is connected.

## `mserve_description`

Robot model for TF, RViz, Gazebo, and Nav2.

Start simple:

```text
mserve_description/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   └── mserve.urdf
├── rviz/
│   └── mserve.rviz
└── test/
    └── test_urdf_load.py
```

Only introduce Xacro files when repetition becomes painful.

## `mserve_bringup`

Top-level launch package.

```text
mserve_bringup/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── mserve_min.launch.py
│   ├── mserve_sim.launch.py
│   ├── mserve_nav.launch.py
│   ├── mserve_hardware.launch.py
│   └── containers.launch.py
└── test/
    └── test_launch_descriptions.py
```

Launch should make topology obvious, following Lesson 10.

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

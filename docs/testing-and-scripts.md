# Testing and Scripts

## Script Shape

Scripts should encode build order and hygiene, not hide ROS 2. The section
below was an early plan for a numbered, phase-based layout — the actual
`scripts/` folder ended up flat and topic-named instead (easier to scan than
hunting through numbered phase folders). Current layout, see
`scripts/README.md` for the full reference:

```text
scripts/
├── run_stack.sh        run_rosbridge.sh    run_web_only.sh    clean_all.sh
├── setup/               env_setup.sh, deps_setup.sh
├── build/               build_workspace.sh, build_packages.sh
├── remote/              launch_remote_rviz.sh, launch_remote_rviz_zenoh.sh,
│                        launch_remote_teleop.sh, start_zenoh_router.sh
├── sim/                 launch_mserve_description_gazebo.sh,
│                        launch_mserve_description_rviz.sh,
│                        stop_mserve_description_gazebo.sh
├── docker/              docker_build_workspace.sh, docker_launch_mserve.sh,
│                        docker_webbridge.sh
└── test/                run_tests.sh
```

## Build Order

1. `mserve_interfaces`
2. `mserve_utils`
3. Core nodes: `mserve_base`, `mserve_esp32`, `mserve_description`, `mserve_bringup`
4. Sensors/display
5. Simulation
6. Navigation
7. Future AI/manipulation

## Unit Test Targets

Use GTest for ROS-free logic:

- Config validation.
- QoS profile parsing.
- Differential-drive kinematics.
- Drive command limits.
- ESP32 packet codec.
- ESP32 timeout/fail-safe behavior.
- Display state transitions.
- Sensor health logic.

## Integration Test Targets

Use `launch_testing` for ROS graph behavior:

- Launch descriptions load.
- Lifecycle nodes reach expected states.
- Configure/activate/deactivate works.
- Topic names match central config.
- ESP32 dry-run feedback appears when wheel commands are published.
- Simulation starts and exposes expected topics.

## Debug Helpers

`debug_ros_graph.sh` should print:

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
ros2 lifecycle nodes
```

Later it can include TF checks:

```bash
ros2 run tf2_tools view_frames
```

## C++ Package Pattern

Each C++ package should follow:

```text
package_name/
├── CMakeLists.txt
├── package.xml
├── include/package_name/
│   ├── logic_or_model.hpp
│   └── node.hpp
├── src/
│   ├── node.cpp
│   └── node_main.cpp
└── test/
    ├── test_logic.cpp
    └── test_integration.py
```

Rules:

- Use C++17.
- Keep header/source separation.
- Keep ROS-free logic separable and unit-tested.
- Use smart pointers.
- Wrap `main()` in `try/catch`, following the lesson style.
- Prefer lifecycle nodes for hardware/simulation subsystems.
- Use callback groups for blocking or long-running work.
- Register composable nodes only after standalone behavior is clear.

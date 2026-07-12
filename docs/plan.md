# mServe ROS 2 C++ Skeleton Plan

This is the short project plan. Detailed design notes live in `docs/` so the root stays easy to scan.

mServe is a learning-first ROS 2 C++ robot stack. Originally built against ROS 2 Jazzy; as of the July 2026 SD-card migration it runs natively (no Docker) against ROS 2 Lyrical. The skeleton should follow the style of `/home/ecm/ros2-systems-operability/src/2_cpp`: clear package boundaries, central configuration, lifecycle nodes, focused tests, and launch files that make topology visible.

## Design Direction

- Build the core robot in C++.
- Keep central YAML configuration in `mserve_interfaces`.
- Use lifecycle nodes for hardware-facing and motion-related subsystems.
- Avoid `ros2_control` at the beginning so motor control, safety limits, odometry boundaries, and ESP32 communication are visible and learnable.
- Keep robot description simple. Use URDF directly where practical; use Xacro only when it removes real duplication without hiding the robot structure.
- Add Gazebo and Nav2 after the minimal skeleton builds and launches.
- Reserve space for AI and an arm, but keep them outside the first critical drive loop.

## Docs Map

- [docs/README.md](docs/README.md): documentation index.
- [docs/architecture.md](docs/architecture.md): project philosophy, control approach, ESP32 boundary, and model rules.
- [docs/packages.md](docs/packages.md): package-by-package skeleton.
- [docs/milestones.md](docs/milestones.md): staged implementation plan.
- [docs/testing-and-scripts.md](docs/testing-and-scripts.md): build, test, utility script plan.
- [docs/TODO.md](docs/TODO.md): running task tracker.
- [docs/session.md](docs/session.md): session notes and decisions.

## First Implementation Pass

Create only enough to get a small, working learning loop:

```text
scripts/
ws/src/mserve_interfaces/
ws/src/mserve_utils/
ws/src/mserve_description/
ws/src/mserve_bringup/
ws/src/mserve_base/
ws/src/mserve_esp32/
```

This first pass should prove:

1. Interfaces and central config build.
2. C++ utility unit tests run.
3. A lifecycle base node starts and can be configured/activated.
4. ESP32 motor comms has a package boundary, even if it begins as a stub.
5. Bringup launch starts the minimal system.
6. The robot description exposes the base, wheels, sensors, display, and future arm mount.

After that, add lidar, camera, display, Gazebo, Nav2, AI, and manipulation one layer at a time.

## Current Open Questions

1. Which ESP32 transport should the first comms stub target: USB serial, UART, UDP, or another link?
2. ~~Which Gazebo stack should we use first?~~ **Decided:** Gazebo Harmonic (`ros_gz`). Originally ran on WSL2/PC; as of the simulation migration, Gazebo **and** RViz both run on the NVIDIA Thor instead (not a PC/WSL2). Pi 5 has no compute GPU and stays hardware-only — see `docs/simulation_hil.md`.
3. What lidar, camera, and display hardware are likely?
4. Should the first AI milestone be perception-only, or high-level task planning?
5. Should the future arm stay as a reserved URDF frame for now, or should `mserve_manipulation` be scaffolded early?

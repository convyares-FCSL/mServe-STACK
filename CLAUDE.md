# mServe-STACK — Claude Code project instructions

## Project

mServe is a learning-first ROS 2 C++ differential-drive robot stack running on a
Raspberry Pi 5. Full design philosophy: `readme.md` and `docs/architecture.md`.

## Runtime environment (as of the 2026-07-18 platform revert)

- Runs in **Docker** (image `mserve-robot:jazzy`, ROS 2 **Jazzy**) on fresh
  Raspberry Pi OS — no native ROS install on this Pi at all.
  `docker-compose.yml` builds `Dockerfile` (`FROM ros:jazzy-ros-base`).
  `run_stack.sh` auto-detects this (`command -v ros2` fails) and routes
  every command through `docker compose exec robot-mserve`.
- Boots via `mserve-drivechain.service` (systemd, `Requires=docker.service`,
  runs as user `ecm`, `ExecStart=scripts/run_stack.sh`). A `boot_splash.py`
  `ExecStartPre` paints an initial state (closed eyes + IP) to `/dev/fb0`
  before the container is even up.
- Gazebo + RViz simulation run on the NVIDIA Thor — not a PC/WSL2, and not the Pi
  itself (Pi 5 has no compute GPU and stays hardware-only).

## Project layout

```
ws/src/
  interfaces/           messages, services, central YAML config (rosidl)
  utils/                shared C++ helpers: params, QoS profiles, topic names
  mserve_base/          command arbiter + safety clamp lifecycle node
  mserve_drivechain/    diff-drive kinematics + JSON/UART link to the ESP32 motor controller
  mserve_camera/        lifecycle node, USB webcam (wraps v4l2_camera's device class)
  mserve_lidar/         lifecycle node, RPLIDAR C1 (vendored SDK, no apt package exists)
  mserve_display/       ELEGOO 3.5" SPI touchscreen UI (plain node, not lifecycle-managed)
  mserve_description/   URDF robot model
  launch/               launch files (mserve_min.launch.py, mserve_slam.launch.py)
  lifecycle_manager/    BT.CPP-driven configure/activate/shutdown for drivechain/base
```

`template/` (an old `hyfleet_subsystem` scaffold from an unrelated project,
left in the tree) carries `COLCON_IGNORE` and is excluded from the build.
(`mserve_base_archive/` — previously also excluded — has since been deleted
outright, not just ignored.) `lifecycle_manager/` is a required runtime
package (drives `mserve_drivechain`/`mserve_base` through configure/activate
via BT.CPP, wired into `launch/mserve_min.launch.py`) and must be built like
any other package.

## Hardware chain

`mserve_base` (`/cmd_vel` → clamp → `/mserve/cmd_vel_safe`) → `mserve_drivechain`
(diff-drive kinematics → JSON over UART on `/dev/ttyAMA0`, the Pi 5 GPIO header) →
onboard ESP32 on a Waveshare DDSM Driver HAT → DDSM115 hub motors ×2. The ESP32
owns the raw DDSM115 binary protocol; the Pi side only ever speaks JSON
(`mserve_drivechain/src/drivechain_uart.cpp`). ESP32 firmware lives in
`ws/src/mserve_drivechain/drive_firmware/`.

Both `mserve_base` and `mserve_drivechain` are lifecycle nodes driven by
BehaviorTree.CPP trees (`src/trees/*.xml`).

## Build

Runs inside the Docker container — `run_stack.sh` does this automatically on
every invocation:

```bash
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --packages-select interfaces utils mserve_drivechain mserve_base \
  launch mserve_description lifecycle_manager btcpp_ros2_interfaces \
  behaviortree_ros2 mserve_camera mserve_lidar mserve_display \
  --cmake-args -DBUILD_TESTING=OFF --symlink-install
```

`utils`, `mserve_drivechain`, `mserve_base`, `mserve_camera`, `mserve_lidar`,
`mserve_display`, and `lifecycle_manager` all use `target_link_libraries()`
with modern imported targets (`rclcpp::rclcpp`, `<pkg>::<pkg>` aggregate
targets for message packages, auto-generated via
`rosidl_cmake_aggregate_target-extras.cmake`, no extra `find_package` needed)
rather than `ament_target_dependencies()` — a portability choice, not a
distro requirement (Jazzy still has the macro). Keep new packages consistent
with this pattern.

## Run

```bash
./scripts/run_stack.sh          # hardware, /dev/ttyAMA0
./scripts/run_stack.sh --sim    # sim backend, no hardware needed
```

Web UI: `http://<pi-ip>:6240/drivechain.html`, `http://<pi-ip>:6240/base.html`.
rosbridge on `:9090`.

## Key decisions (full list in readme.md "Design decisions")

- `mserve_base` does not own kinematics — that's `mserve_drivechain`'s job, so
  swapping drivetrain hardware only touches one package.
- No `ros2_control` yet — the first control stack is hand-written (clamping,
  kinematics, protocol, fail-safe) so every part is visible and learnable.
- Parameter bounds are enforced by ROS descriptors, not manual throws.

## Docs caveat

`docs/architecture.md`, `docs/packages.md`, `docs/milestones.md`, `docs/plan.md`
are early planning docs. Some describe a `mserve_esp32` **ROS package** boundary
that was ultimately implemented differently — the ESP32 is a physical board
running its own firmware (`mserve_drivechain/drive_firmware/`), not a separate
ROS package — and use package names (`mserve_interfaces`, `mserve_bringup`) that
don't match the current `interfaces`/`launch` folders. Treat those docs as
historical intent, not current fact; check actual `ws/src/` structure and the
per-package READMEs (e.g. `ws/src/mserve_drivechain/README.md`, which is
detailed and current) first.

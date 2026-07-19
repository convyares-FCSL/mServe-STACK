# mServe-STACK — Claude Code project instructions

## Project

mServe is a learning-first ROS 2 C++ differential-drive robot stack running on a
Raspberry Pi 5. Full design philosophy: `readme.md` and `docs/architecture.md`.

## Runtime environment

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
  mserve_base/          safety clamp + diff-drive kinematics + odometry lifecycle node (future command arbiter)
  mserve_drivechain/    pure motor driver — JSON/UART link to the ESP32 motor controller
  mserve_camera/        lifecycle node, USB webcam (wraps v4l2_camera's device class)
  mserve_lidar/         lifecycle node, RPLIDAR C1 (vendored SDK, no apt package exists)
  mserve_display/       ELEGOO 3.5" SPI touchscreen UI (plain node, not lifecycle-managed)
  mserve_joystick/      game controller teleop, consumes /joy (plain node, not lifecycle-managed)
  mserve_sensehat/      Pi Sense HAT: LED matrix, joystick, IMU (vendored RTIMULib, plain node)
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

`mserve_base` (`/cmd_vel` → clamp → diff-drive kinematics → per-motor RPM via
`interfaces/srv/Drive`; also publishes `/mserve/cmd_vel_safe` and IMU-fused
`/odom`) → `mserve_drivechain`
(pure motor driver — JSON over UART on `/dev/ttyAMA0`, the Pi 5 GPIO header) →
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
  behaviortree_ros2 mserve_camera mserve_lidar mserve_display mserve_joystick \
  mserve_sensehat \
  --cmake-args -DBUILD_TESTING=OFF --symlink-install
```

`utils`, `mserve_drivechain`, `mserve_base`, `mserve_camera`, `mserve_lidar`,
`mserve_display`, `mserve_sensehat`, and `lifecycle_manager` all use `target_link_libraries()`
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

- `mserve_drivechain` does not own kinematics — wheel geometry and diff-drive
  math (both directions, including odometry) live in `mserve_base`; drivechain
  is a pure motor driver (per-motor RPM in, feedback out), so swapping
  drivetrain hardware only touches one package.
- No `ros2_control` yet — the first control stack is hand-written (clamping,
  kinematics, protocol, fail-safe) so every part is visible and learnable.
- Parameter bounds are enforced by ROS descriptors, not manual throws.
- Optional USB/input peripherals (camera, lidar, touch) are live directory
  bind-mounts in `docker-compose.yml` (`/dev/host` = the whole host `/dev`,
  plus `/dev/input`), not static `devices:` entries — a missing/unplugged
  peripheral no longer blocks the *entire* container from starting (it used
  to: an unplugged RPLIDAR once crash-looped the whole systemd service, not
  just `mserve_lidar`). By-id paths resolve fresh on every `open()` instead
  of being snapshotted at container-create time, so `--force-recreate` isn't
  needed on every `run_stack.sh` run either.

## Verification

For display/hardware-adjacent fixes, don't declare done from code review or
compile success alone — verify empirically on the real device, then report
what was actually observed, not what was expected. Several fixes to
`mserve_display` looked correct by reasoning alone but were subtly wrong on
real hardware: a screen-orientation fix that needed a full 180-degree
rotation, not the more "logical" Y-only flip; an eye-direction sign bug that
only surfaced by actually driving via the web UI's buttons, not a synthetic
`ros2 topic pub`; an eyebrow color that was technically present in the
framebuffer but visually invisible against the background. For display work
specifically, dump and view the actual rendered output (`/dev/fb0` → PPM →
PNG) rather than trusting the code path alone. After deploying a fix,
restart the actual running node/container and re-verify — a rebuilt binary
isn't running until the process is restarted.

## Docs

Three living docs plus per-package READMEs — nothing else:
`docs/operations.md` (full run/build/debug reference), `docs/architecture.md`
(philosophy, boundaries, C++ conventions), `docs/TODO.md` (statements of
what's next; history lives in `git log`, not in docs). Per-package READMEs
(e.g. `ws/src/mserve_drivechain/README.md`) are the source of truth for each
package. The planning-era docs (`plan.md`, `packages.md`, `milestones.md`,
`session.md`, etc.) were deleted 2026-07-19 — don't recreate them; put
what's-next in TODO.md, decisions/rationale in the nearest README or config
comment, and nothing in a changelog.

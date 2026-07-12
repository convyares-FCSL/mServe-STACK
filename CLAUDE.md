# mServe-STACK — Claude Code project instructions

## Project

mServe is a learning-first ROS 2 C++ differential-drive robot stack running on a
Raspberry Pi 5. Full design philosophy: `readme.md` and `docs/architecture.md`.

## Runtime environment (as of the July 2026 SD-card migration)

- Runs **natively** on ROS 2 **Lyrical** (Ubuntu 26.04 "Resolute") — no Docker.
  `Dockerfile`/`docker-compose.yml` are legacy fallback only; `run_stack.sh`
  uses them automatically only if `ros2` isn't found on PATH.
- Originally built against ROS 2 Jazzy. If building from scratch on a newer distro
  and `ament_target_dependencies()` errors as an unknown CMake command, see Build below.
- Boots via `mserve-drivechain.service` (systemd, native — sources
  `/opt/ros/lyrical/setup.bash` + `ws/install/setup.bash`, then runs
  `scripts/run_stack.sh`).
- Gazebo + RViz simulation run on the NVIDIA Thor — not a PC/WSL2, and not the Pi
  itself (Pi 5 has no compute GPU and stays hardware-only).

## Project layout

```
ws/src/
  interfaces/           messages, services, central YAML config (rosidl)
  utils/                shared C++ helpers: params, QoS profiles, topic names
  mserve_base/          command arbiter + safety clamp lifecycle node
  mserve_drivechain/    diff-drive kinematics + JSON/UART link to the ESP32 motor controller
  mserve_description/   URDF robot model
  launch/               launch files (mserve_min.launch.py)
```

`mserve_base_archive/`, `template/` (an old `hyfleet_subsystem` scaffold from an
unrelated project, left in the tree), and `lifecycle_manager/` all carry
`COLCON_IGNORE` and are excluded from the build.

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

```bash
source /opt/ros/lyrical/setup.bash
cd ws
colcon build --packages-select interfaces utils mserve_drivechain mserve_base \
  --cmake-args -DBUILD_TESTING=OFF --symlink-install
```

`ament_target_dependencies()` does not exist in Lyrical (fully removed, not just
deprecated). Use `target_link_libraries()` with modern imported targets instead —
`rclcpp::rclcpp`, and `<pkg>::<pkg>` aggregate targets for message packages
(auto-generated via `rosidl_cmake_aggregate_target-extras.cmake`, no extra
`find_package` needed). Already fixed in the current CMakeLists for `utils`,
`mserve_drivechain`, `mserve_base` — keep new packages consistent with this pattern.

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

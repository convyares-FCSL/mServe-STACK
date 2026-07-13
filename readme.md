# mServe Stack

ROS 2 C++ robot stack for the mServe differential-drive robot, running natively (no Docker) on a Raspberry Pi 5. Originally built against ROS 2 Jazzy; as of the July 2026 SD-card migration it builds and runs natively against ROS 2 Lyrical — see the CMake note in [Build](#build) if you're building from scratch on a newer distro.

The full stack — drivechain, base, camera, lidar — starts automatically on boot via systemd (`mserve-drivechain.service`), so the robot is ready as soon as the Pi powers on.

The design philosophy is learning-first: every control boundary, kinematic calculation, and hardware protocol is written by hand before reaching for frameworks like `ros2_control`. Each layer should be readable on its own.

## Hardware

- Raspberry Pi 5 (runtime)
- DDSM115 hub motors × 2, driven via a Waveshare DDSM Driver HAT — its onboard ESP32 speaks JSON over UART on the Pi 5 GPIO header (`/dev/ttyAMA0`) and handles the raw DDSM115 protocol itself; the ESP32 firmware lives in `ws/src/mserve_drivechain/drive_firmware/`
- Generic USB UVC webcam (`/dev/video0`) — driven by `mserve_camera`
- SLAMTEC RPLIDAR C1, USB-serial (`/dev/ttyUSB0`, 460800 baud) — driven by `mserve_lidar`
- Gazebo + RViz simulation run on the NVIDIA Thor — the Pi 5 has no compute GPU and stays hardware-only

## Workspace

```
ws/src/
  interfaces/            central config, messages, services
  utils/                 shared utilities: params, QoS profiles, topic names
  mserve_base/           command arbiter and safety clamp
  mserve_drivechain/     diff-drive kinematics + JSON/UART protocol to the ESP32 motor controller
  mserve_camera/         lifecycle node wrapping v4l2_camera's device class directly (USB webcam)
  mserve_lidar/          lifecycle node wrapping a vendored RPLIDAR SDK directly (RPLIDAR C1)
  mserve_description/    URDF robot model
  lifecycle_manager/     BehaviorTree.CPP-driven configure/activate/shutdown of all four nodes
  launch/                launch files
  third_party/           vendored source deps (BehaviorTree.ROS2, slam_toolbox) — gitignored, see third_party/README.md
```

(Folder names dropped the `mserve_` prefix on `interfaces`/`utils`/`launch` at some point — the docs below still use the old prefixed names in places; treat the folder names above as current.)

### Package responsibilities

| Package | Role |
|---------|------|
| `interfaces` | Shared messages, services, central YAML config |
| `utils` | `get_or_declare_param`, `bounded_double`, named QoS profiles, topic name utilities |
| `mserve_base` | Subscribes `/cmd_vel`, clamps to speed limits, publishes `/mserve/cmd_vel_safe` |
| `mserve_drivechain` | Subscribes `/mserve/cmd_vel_safe`, runs diff-drive kinematics, owns the JSON-over-UART link to the onboard ESP32 (Waveshare DDSM Driver HAT) — the ESP32 itself handles the raw DDSM115 protocol |
| `mserve_camera` | Wraps `v4l2_camera`'s `V4l2CameraDevice` directly (not the whole `v4l2_camera_node`), publishes `sensor_msgs/Image` + `CameraInfo` — see `ws/src/mserve_camera/README.md` |
| `mserve_lidar` | Wraps a vendored Slamtec RPLIDAR SDK directly (no apt package exists for `rplidar_ros` on this distro), publishes `sensor_msgs/LaserScan` — see `ws/src/mserve_lidar/README.md` |
| `mserve_description` | URDF robot description |
| `lifecycle_manager` | BT-driven configure/activate on bringup, deactivate/shutdown on SIGINT, for all four lifecycle nodes above — see `ws/src/lifecycle_manager/README.md` |
| `launch` | `mserve_min.launch.py` — starts drivechain + base + camera + lidar + `robot_state_publisher` + `lifecycle_manager` together |

## Files

- `.env`: local runtime settings (gitignored).
- `Dockerfile` / `docker-compose.yml`: legacy — the stack ran in Docker before the July 2026 migration to the Ubuntu 26.04 SD card. Left in place for reference but unused; `run_stack.sh` only falls back to them if it can't find `ros2` on PATH.
- `docs/`: design notes, milestones, session log, task tracker.
- `scripts/`: all helper scripts, flat and topic-named (`scripts/README.md` has the full list).
- `web/`: debug browser UI (lifecycle control + cmd_vel publisher), served by `scripts/run_stack.sh`.
- `ws/`: ROS 2 workspace (`ws/src/`), built natively with `colcon`.

## Docs

- `docs/plan.md` — short project plan and open questions.
- `docs/architecture.md` — control philosophy, ESP32 boundary, lifecycle rules.
- `docs/packages.md` — package-by-package design notes.
- `docs/milestones.md` — staged build path.
- `docs/TODO.md` — task tracker.
- `docs/session.md` — session decisions and notes.
- `docs/testing-and-scripts.md` — test strategy and script reference.
- `docs/simulation_hil.md` — simulation status and hardware-in-loop plan.
- `docs/remote-rviz-zenoh.md` — running RViz on a remote machine (e.g. Thor) over Zenoh; current procedure.

`docs/plan.md`, `docs/architecture.md`, `docs/packages.md`, and
`docs/milestones.md` are early planning docs and describe some package
boundaries/names that changed once actually built (e.g. a planned
`mserve_esp32` ROS package that became ESP32 firmware instead, or
`mserve_interfaces`/`mserve_bringup` vs. the current `interfaces`/`launch`
folder names) — treat them as historical intent, check `ws/src/` and the
per-package READMEs for current fact.

## Normal flow

The full stack (rosbridge + `mserve_drivechain` + `mserve_base` + `mserve_camera` + `mserve_lidar` + web UI) starts automatically on boot via `mserve-drivechain.service` (see [Running on boot](#running-on-boot-systemd)) — **on a running Pi you normally don't need to start anything by hand, just open the web UI directly:**

- `http://<pi-ip>:6240/drivechain.html`
- `http://<pi-ip>:6240/base.html`
- `http://<pi-ip>:6240/camera.html`
- `http://<pi-ip>:6240/lidar.html`

e.g. on this Pi (static Wi-Fi IP): `http://172.16.68.73:6240/drivechain.html` —
also reachable over Tailscale at `http://100.122.150.74:6240/...` from off-network.

## Running manually

Use this instead of the boot service after stopping it, or for development.
Source ROS 2 first, then start the stack:

```bash
source /opt/ros/lyrical/setup.bash
./scripts/run_stack.sh              # hardware, /dev/ttyAMA0 (Pi 5 GPIO UART)
./scripts/run_stack.sh --sim        # simulated backend, no hardware needed
./scripts/run_stack.sh /dev/ttyACM0 # hardware, custom UART device (e.g. USB)
```

This expects the workspace already built (falls back to the legacy
`robot-mserve` Docker container only if `ros2` isn't found on PATH). It
starts rosbridge, then launches `mserve_drivechain` + `mserve_base` +
`mserve_camera` + `mserve_lidar` + `robot_state_publisher` +
`lifecycle_manager` together via `ws/src/launch/launch/mserve_min.launch.py`
— `lifecycle_manager` drives configure/activate for all four lifecycle
nodes, not the script itself — and serves the debug UI at the same URLs
above once drivechain and base report `active`.

The UI shows lifecycle state for each node and allows
configure/activate/deactivate/shutdown and cmd_vel publishing. Click
**Connect** on the drivechain page before driving — activation alone doesn't
open the UART port, `~/connect` does.

Press Ctrl+C to stop. This runs `lifecycle_manager`'s shutdown tree
(deactivates all four nodes) before tearing down rosbridge/web server cleanly.

## Remote RViz (Zenoh)

Gazebo + RViz run on the NVIDIA Thor, not the Pi (see [Hardware](#hardware)).
Two scripts, one per machine, in `scripts/remote/`:

```bash
# On the Pi — start the router, leave it running:
source /opt/ros/lyrical/setup.bash
./scripts/remote/start_zenoh_router.sh

# On Thor — point RViz at the Pi's LAN or Tailscale address:
./scripts/remote/launch_remote_rviz_zenoh.sh 172.16.68.73     # LAN
./scripts/remote/launch_remote_rviz_zenoh.sh 100.122.150.74   # Tailscale, off-LAN
```

Order doesn't matter — the router and `run_stack.sh` are independent
processes. See **[`docs/remote-rviz-zenoh.md`](docs/remote-rviz-zenoh.md)**
for why Zenoh instead of plain DDS discovery, what each script actually sets
up, and troubleshooting.

## Running on boot (systemd)

`mserve-drivechain.service` (native — no Docker dependency) sources ROS 2 Lyrical + the workspace and runs `./scripts/run_stack.sh` on boot, so the robot comes up ready on power-on.

```bash
sudo systemctl status mserve-drivechain      # check it's running
sudo journalctl -u mserve-drivechain -f      # live logs
sudo systemctl restart mserve-drivechain     # restart the stack
sudo systemctl stop mserve-drivechain        # stop everything (runs cleanup)
```

`stop`/`restart` send SIGTERM to the script, triggering the same cleanup as Ctrl+C.

## Build

`lifecycle_manager` depends on `behaviortree_ros2`/`btcpp_ros2_interfaces`,
vendored (not apt-installed) at `ws/src/third_party/BehaviorTree.ROS2/` —
clone + build once before the first build of the main stack. `slam_toolbox`
(also vendored, `ws/src/third_party/slam_toolbox/`) is only needed if you're
using `scripts/run_slam.sh`, not for the core drive stack. Full clone/apt/
build commands for both: **[`ws/src/third_party/README.md`](ws/src/third_party/README.md)**
— or just run `scripts/setup/deps_setup.sh`, which does all of it.

`mserve_camera` and `mserve_lidar` need their own system deps — camera wraps
an apt-installed driver package, lidar vendors its SDK as source (no apt
package for it exists on this distro), and the web UI needs rosbridge +
image transcoding:

```bash
sudo apt install ros-lyrical-v4l2-camera ros-lyrical-rosbridge-server ros-lyrical-web-video-server
```

Then, from `/home/ecm/mServe-STACK/ws`, natively (no Docker), build
everything the stack actually launches:

```bash
source /opt/ros/lyrical/setup.bash
colcon build \
  --packages-select interfaces utils mserve_drivechain mserve_base \
    mserve_camera mserve_lidar mserve_description lifecycle_manager launch \
    btcpp_ros2_interfaces behaviortree_ros2 \
  --cmake-args -DBUILD_TESTING=OFF --symlink-install
```

**CMake note (Jazzy → Lyrical):** `ament_target_dependencies()` was removed
entirely in ROS 2 Lyrical (not just deprecated — the macro doesn't exist).
The CMakeLists for `utils`, `mserve_drivechain`, `mserve_base`,
`mserve_camera`, `mserve_lidar`, and `lifecycle_manager` all use
`target_link_libraries()` with modern
imported targets instead (`rclcpp::rclcpp`, `interfaces::interfaces`, etc. —
message packages export a `<pkg>::<pkg>` aggregate target automatically via
`rosidl_cmake_aggregate_target-extras.cmake`, no extra `find_package` needed).
If you're building this from scratch on an even newer distro and hit
`Unknown CMake command "ament_target_dependencies"`, this is why.

The Docker path (`docker compose build robot-mserve`,
`scripts/docker/docker_build_workspace.sh`) still exists as a fallback —
`run_stack.sh` uses it automatically if `ros2` isn't on PATH — but
is no longer the primary workflow on this Pi.

## Runtime parameter updates (no restart needed)

```bash
# Speed limits hot-swap on both nodes
ros2 param set /mserve_base limits.max_linear_speed 0.5
ros2 param set /mserve_drivechain feedback_rate 10.0

# Camera/lidar frame_id also hot-swaps; device/baudrate/width/height don't —
# rejected unless the node is unconfigured first (reopening a device mid-stream isn't supported)
ros2 param set /mserve_lidar frame_id lidar_link

# Describe a parameter to see its valid range
ros2 param describe /mserve_base limits.max_linear_speed
```

## Stop / remove

To stop the drive stack, use `sudo systemctl stop mserve-drivechain` (or Ctrl+C if running `run_stack.sh` manually) — see [Running on boot](#running-on-boot-systemd). Since everything runs as native processes now, `stop` is the whole story — there's no container to separately remove.

## Logs

```bash
sudo journalctl -u mserve-drivechain -f
```

rosbridge's own log is written to `/tmp/rosbridge.log` (see `run_stack.sh`).

Grafana/Loki (if running):

```logql
{robot="mserve"}
```

## Design decisions

- **`mserve_drivechain` does not own kinematics.** Wheel geometry (`wheel_separation`, `wheel_radius`) and diff-drive math (both directions — `/cmd_vel` → wheel RPM, and wheel velocity feedback → odometry) live in `mserve_base`. `mserve_drivechain` is a pure motor driver — it only ever sees per-motor RPM commands and reports per-motor velocity/position feedback, so swapping the drivetrain hardware only requires changes in one package.
- **`mserve_base` is the future command arbiter.** When Nav2 and manual joystick are added, source arbitration and the e-stop gate live here. `mserve_drivechain` always sees a single safe Twist.
- **Parameter bounds enforced by ROS descriptors.** Out-of-range values are rejected before callbacks are called. No manual throws needed.
- **Named QoS profiles from `mserve_utils`.** `mserve_qos::commands`, `mserve_qos::feedback`, `mserve_qos::status` — profiles readable from params without recompiling.
- **No `ros2_control` yet.** The first control stack is written by hand so every part (clamping, kinematics, serial protocol, odometry, fail-safe) is visible and learnable. `ros2_control` revisited later as a comparison.
- **Hardware nodes wrap the vendor's device/driver class directly, not the vendor's ROS node.** `mserve_drivechain` wraps its own `DriveUart`, `mserve_camera` wraps `v4l2_camera`'s `V4l2CameraDevice`, `mserve_lidar` wraps a vendored RPLIDAR SDK's `sl::ILidarDriver` — never someone else's whole `rclcpp::Node`. Reasoning is the same each time: an upstream driver node is a plain node, not a lifecycle node, so it can't be configured/activated in step with the rest of the stack or driven by `lifecycle_manager`.

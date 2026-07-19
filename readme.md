# mServe Stack

ROS 2 C++ robot stack for the mServe differential-drive robot, running in Docker (ROS 2 Jazzy) on a Raspberry Pi 5. There is no native ROS install on this Pi — Docker is the only runtime path.

Drivechain, base, camera (`mserve_camera`), lidar (`mserve_lidar`), rosbridge,
Foxglove Bridge (on by default, `--no-foxglove` to skip it), SLAM Toolbox
(`--slam-map`/`--slam-local`), Nav2 (`--nav2`, implies `--slam-local` if no
SLAM flag is given), and the full web UI (`drivechain.html`/`base.html`/
`camera.html`/`lidar.html`/`joystick.html`/`sensehat.html`) all run in
Docker. Device paths use udev `by-id` stable
symlinks (see `docker-compose.yml`), not raw `/dev/videoN`/`/dev/ttyUSBN` —
those indices aren't stable across USB resets/replugs on this Pi, confirmed
the hard way. `slam_toolbox` itself needed a small source patch (this
distro's `message_filters` changed API underneath it) — see
`ws/src/third_party/README.md`. Zenoh remote-RViz isn't ported to Docker yet
— same-LAN access is already covered by Foxglove Bridge; see `docs/TODO.md`
if you need it.

The drivechain + base stack starts automatically on boot via systemd (`mserve-drivechain.service`), so the robot is ready as soon as the Pi powers on.

The design philosophy is learning-first: every control boundary, kinematic calculation, and hardware protocol is written by hand before reaching for frameworks like `ros2_control`. Each layer should be readable on its own.

## Hardware

- Raspberry Pi 5 (runtime)
- DDSM115 hub motors × 2, driven via a Waveshare DDSM Driver HAT — its onboard ESP32 speaks JSON over UART on the Pi 5 GPIO header (`/dev/ttyAMA0`) and handles the raw DDSM115 protocol itself; the ESP32 firmware lives in `ws/src/mserve_drivechain/drive_firmware/`
- Generic USB UVC webcam (`/dev/video0`) — driven by `mserve_camera`
- SLAMTEC RPLIDAR C1, USB-serial (`/dev/ttyUSB0`, 460800 baud) — driven by `mserve_lidar`
- ELEGOO 3.5" SPI TFT touchscreen (ILI9486 display + ADS7846 resistive touch) — driven by `mserve_display`, a small status/control UI (face, connect/info/calibrate menu)
- Pi Sense HAT (2): 8x8 LED matrix, 5-way joystick, LSM9DS1 IMU — driven by `mserve_sensehat`. Its framebuffer driver can register ahead of the touchscreen's (bumping it from `/dev/fb0` to `/dev/fb1`) — both display packages resolve their device by driver name at runtime rather than assuming a fixed path, see `mserve_sensehat/README.md`
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
  mserve_display/        plain node (not lifecycle) driving the ELEGOO touchscreen UI
  mserve_joystick/       plain node (not lifecycle), game controller teleop, consumes /joy
  mserve_sensehat/       plain node (not lifecycle), Pi Sense HAT: LED matrix, joystick, IMU
  mserve_description/    URDF robot model
  lifecycle_manager/     BehaviorTree.CPP-driven configure/activate/shutdown of the four lifecycle nodes
  launch/                launch files
  third_party/           vendored source deps (BehaviorTree.ROS2, slam_toolbox, RTIMULib) — gitignored, see third_party/README.md
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
| `mserve_display` | Plain node (not lifecycle-managed) driving the ELEGOO touchscreen — face w/ eyes tracking cmd_vel, connect/info/calibrate menu — see `ws/src/mserve_display/README.md` |
| `mserve_joystick` | Plain node (not lifecycle-managed), game controller teleop — `/joy` → `/cmd_vel` + button actions (connect, display info, speed/angular scale) — see `ws/src/mserve_joystick/README.md` |
| `mserve_sensehat` | Plain node (not lifecycle-managed), Pi Sense HAT — LED matrix shows drivechain connection status, joystick center-click calls `connect`, publishes `sensor_msgs/Imu` via vendored RTIMULib — see `ws/src/mserve_sensehat/README.md` |
| `mserve_description` | URDF robot description |
| `lifecycle_manager` | BT-driven configure/activate on bringup, deactivate/shutdown on SIGINT, for the four lifecycle nodes above (`mserve_display`/`mserve_joystick`/`mserve_sensehat` aren't lifecycle-managed) — see `ws/src/lifecycle_manager/README.md` |
| `launch` | `mserve_min.launch.py` — starts drivechain + base + camera + lidar + display + joystick + sensehat + `robot_state_publisher` + `lifecycle_manager` together |

## Files

- `.env`: local runtime settings (gitignored).
- `Dockerfile` / `docker-compose.yml`: the active runtime (ROS 2 Jazzy). `run_stack.sh` auto-detects and uses them whenever `ros2` isn't found on the host PATH — true on this Pi OS install, which has no native ROS at all.
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
- `http://<pi-ip>:6240/joystick.html`
- `http://<pi-ip>:6240/sensehat.html`

Off-network access is via Raspberry Pi Connect (`rpi-connect` — installed but
not yet signed in, `rpi-connect signin`).

## Running manually

Use this instead of the boot service after stopping it, or for development.
No need to source anything on the host — `run_stack.sh` detects there's no
native `ros2` on PATH and runs everything inside the `robot-mserve` Docker
container automatically (building/rebuilding the workspace inside it first,
every run):

```bash
./scripts/run_stack.sh              # hardware, /dev/ttyAMA0 (Pi 5 GPIO UART) — Foxglove Bridge on by default
./scripts/run_stack.sh --sim        # simulated backend, no hardware needed
./scripts/run_stack.sh /dev/ttyACM0 # hardware, custom UART device (e.g. USB)
./scripts/run_stack.sh --no-foxglove    # skip Foxglove Bridge (ws://<pi-ip>:8765)

./scripts/run_stack.sh --slam-map       # also start SLAM Toolbox, building/extending a map
./scripts/run_stack.sh --slam-local     # also start SLAM Toolbox, localizing against a saved map
./scripts/run_stack.sh --nav2           # + Nav2 — implies --slam-local (no need to pass both)
```

`--sim`, `--no-foxglove`, `--slam-map`/`--slam-local`, and `--nav2` are
order-independent and can be combined (e.g. `--sim --nav2`). `--nav2` needs
a live `map -> odom` transform — if neither `--slam-map` nor `--slam-local`
is also given, it implies `--slam-local` automatically (pass `--slam-map`
explicitly instead if you want Nav2 running while still building the map).
Either way it needs `ws/src/interfaces/config/slam_params_local.yaml`'s
`map_file_name` already pointing at a real saved map (see
[SLAM + Navigation](#slam--navigation) below). SLAM Toolbox needs
`ws/src/third_party/slam_toolbox/` vendored first — see [Build](#build).
Nav2 is apt-installed (`ros-jazzy-navigation2`), nothing to vendor.

It starts rosbridge + `web_video_server`, then launches `mserve_drivechain` +
`mserve_base` + `mserve_camera` + `mserve_lidar` + `robot_state_publisher` +
`lifecycle_manager` together via
`ws/src/launch/launch/mserve_min.launch.py` (`with_camera`/`with_lidar`
default `true` in both native and Docker mode now — `run_stack.sh` doesn't
currently expose a flag to turn them off; edit `LAUNCH_ARGS` in the script
if you need to run without that hardware attached) — `lifecycle_manager`
drives configure/activate for all four lifecycle nodes, not the script
itself — and serves the debug UI at the same URLs above once drivechain and
base report `active`.

The UI shows lifecycle state for each node and allows
configure/activate/deactivate/shutdown and cmd_vel publishing. Click
**Connect** on the drivechain page before driving — activation alone doesn't
open the UART port, `~/connect` does.

Press Ctrl+C to stop. This runs `lifecycle_manager`'s shutdown tree
(deactivates drivechain + base, plus camera + lidar if running natively with
them enabled) before tearing down rosbridge/web server cleanly.

## SLAM + Navigation

`slam_toolbox` (`--slam-map`/`--slam-local`) and Nav2 (`--nav2`) are fully
independent — `slam_toolbox`'s only job is *building* maps; Nav2 localizes
against a saved one itself, via AMCL + `map_server`, not `slam_toolbox`'s
own localization mode (switched 2026-07-19 — see `nav2_params.yaml`'s
header comment: `slam_toolbox`'s localization mode has no equivalent of
AMCL's `/initialpose`, so there was no quick way to correct the believed
pose after moving the robot by hand, only a clunky `deserialize_map`
service call with a manually typed-in pose).

Full workflow, mapping through autonomous navigation:

```bash
# 1. Map a space (drive around manually — web UI, joystick, or Sense HAT):
./scripts/run_stack.sh --slam-map
# Watch /map build live in Foxglove (on by default). When done, save it as a
# plain image (what Nav2's map_server needs) — NOT serialize_map, that's a
# different format (a pose-graph, only relevant if you want slam_toolbox's
# own --slam-local mode for something other than Nav2):
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/root/mServe-STACK/map/my_map'}}"

# 2. Point nav2_params.yaml's map_server at it (one-time, per map):
#    ws/src/interfaces/config/nav2_params.yaml -> map_server.yaml_filename: /root/mServe-STACK/map/my_map.yaml

# 3. Localize (AMCL) and drive Nav2:
./scripts/run_stack.sh --nav2
```

**If the robot's real position doesn't match what it believes** (moved by
hand, kidnapped-robot situation, or just started somewhere other than the
map's origin) — use Foxglove/RViz's **2D Pose Estimate** tool: click where
the robot actually is, drag to set its actual facing direction, release.
That publishes `geometry_msgs/PoseWithCovarianceStamped` to `/initialpose`,
which AMCL uses directly to re-seed its particle filter — no service calls
needed, this is the whole reason AMCL was chosen over `slam_toolbox`'s own
localization mode for this role.

**Sending a Nav2 goal from Foxglove**: `bt_navigator` subscribes to
`/goal_pose` (`geometry_msgs/msg/PoseStamped`) directly — no action client
needed, same simple-goal bridge RViz's "2D Nav Goal" button uses. In
Foxglove's 3D panel: set **Fixed Frame to `map`** (not `odom` — goals need
map-frame coordinates), open the panel's Publish settings and set the
"Pose" publish topic to `/goal_pose` (Foxglove defaults this to the ROS1-era
`/move_base_simple/goal`, which nothing here listens to), then use the
3D view's publish-pose tool: click where you want the robot to go, drag to
set the facing direction, release to send. Or from the CLI:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

Nav2 (`ws/src/launch/launch/mserve_nav2.launch.py`, params in
`ws/src/interfaces/config/nav2_params.yaml`) runs `map_server` + `amcl` for
localization plus the planning/control layer (`controller_server` +
`planner_server` + `behavior_server` + `bt_navigator`), all managed by
Nav2's own `nav2_lifecycle_manager` — a separate thing from this project's
own BT.CPP-driven `lifecycle_manager`, which only manages mserve's own
packages. Both costmaps use a `footprint` polygon (real measured chassis:
370mm long, 315mm wheel-to-wheel, plus a 20mm pad), not a `robot_radius`
circle — a circle sized to the chassis diagonal left almost no margin in
interior doorways, since it penalizes every heading by the full diagonal
even when driving straight through. `inflation_radius: 0.15` on both
costmaps; needed the tighter footprint first to afford any inflation at
all without reblocking doorways (see `nav2_params.yaml`'s own comments for
the full doorway-tuning story, 2026-07-19). AMCL's odometry noise model
(`alpha1`-`5`) is left at Nav2's own untuned defaults. Velocity caps are
deliberately conservative for a first-tested integration; raise once
trusted on real hardware.

## Remote RViz (Zenoh)

Gazebo + RViz run on the NVIDIA Thor, not the Pi (see [Hardware](#hardware)).
For same-LAN viewing/teleop without any of this, Foxglove Bridge (on by
default, above) is the simpler path — Zenoh is only needed for discovery
across Tailscale or other multi-interface setups where DDS/plain multicast
discovery doesn't reliably work.

**Deferred, not yet ported to Docker** (tracked in `docs/TODO.md`'s
DEFERRED section) — the scripts below assume a native `ros2` on PATH, which
this Pi OS install doesn't have. Likely needs `network_mode: host` in
`docker-compose.yml` since DDS/Zenoh multicast discovery doesn't traverse
Docker's default bridge network cleanly — a real decision, not copy-paste.
Two scripts, one per machine, in `scripts/remote/`:

```bash
# On the Pi — start the router, leave it running (native ROS 2 required):
source /opt/ros/jazzy/setup.bash
./scripts/remote/start_zenoh_router.sh

# On Thor — point RViz at the Pi's LAN or Tailscale address:
./scripts/remote/launch_remote_rviz_zenoh.sh 172.16.68.73     # LAN
./scripts/remote/launch_remote_rviz_zenoh.sh 100.122.150.74   # Tailscale, off-LAN
```

Order doesn't matter — the router and `run_stack.sh` are independent
processes. `rmw_zenoh_cpp` sidesteps DDS's reliance on multicast: the Pi
runs a Zenoh router at a known address, and the remote machine connects to
it directly (`client` mode) instead of relying on multicast/gossip to find
it. The remote-side script sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` +
`ZENOH_CONFIG_OVERRIDE`, restarts its local `ros2 daemon` (it caches
discovery config from its last start), and opens RViz pre-configured with
`ws/src/mserve_description/rviz/mserve.rviz`.

## Running on boot (systemd)

`mserve-drivechain.service` (`Requires=docker.service` — checked into the repo at `systemd/mserve-drivechain.service`, install with `sudo cp systemd/mserve-drivechain.service /etc/systemd/system/ && sudo systemctl daemon-reload`) runs `./scripts/run_stack.sh` on boot, which starts the Docker container and builds/launches the workspace inside it, so the robot comes up ready on power-on.

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

`mserve_camera` and `mserve_lidar`'s system deps (camera's apt-installed
driver package, lidar's SDK build tools) and the web UI's rosbridge/image
transcoding are all baked into `Dockerfile` already — nothing to install by
hand on this Pi.

`run_stack.sh` does the build automatically inside the `robot-mserve` Docker
container on every run (`docker compose build robot-mserve` once to build
the image itself, first, which `docker compose up` does automatically if the
image doesn't exist yet). The equivalent manual build, run inside the
container (`docker compose exec robot-mserve bash`, or see what `run_stack.sh`
itself runs):

```bash
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build \
  --packages-select interfaces utils mserve_drivechain mserve_base \
    mserve_camera mserve_lidar mserve_display mserve_joystick mserve_sensehat \
    mserve_description lifecycle_manager launch btcpp_ros2_interfaces \
    behaviortree_ros2 \
  --cmake-args -DBUILD_TESTING=OFF --symlink-install
```

`utils`, `mserve_drivechain`, `mserve_base`, `mserve_camera`, `mserve_lidar`,
`mserve_display`, `mserve_sensehat`, and `lifecycle_manager` all use `target_link_libraries()`
with modern imported targets (`rclcpp::rclcpp`, `interfaces::interfaces`,
etc. — message packages export a `<pkg>::<pkg>` aggregate target
automatically via `rosidl_cmake_aggregate_target-extras.cmake`, no extra
`find_package` needed) rather than `ament_target_dependencies()` — a
portability choice, not a distro requirement.

If you're building on a machine with ROS 2 installed natively instead of in
Docker, the same `colcon build` command above works unchanged from that
machine's workspace root — just make sure `mserve_camera`/`mserve_lidar`'s
system deps (see `Dockerfile` for the exact apt package list) are installed
first.

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

To stop the drive stack, use `sudo systemctl stop mserve-drivechain` (or Ctrl+C if running `run_stack.sh` manually) — see [Running on boot](#running-on-boot-systemd). The `robot-mserve` container is started with a plain `docker compose up -d` and left running across `run_stack.sh` invocations — camera/lidar/touch are live directory bind-mounts (`/dev/v4l`, `/dev/serial`, `/dev/input`), not `devices:` entries, so by-id paths resolve fresh on every open() rather than needing a container recreate to re-resolve (see `docker-compose.yml`'s `devices:`/`volumes:` comments). `docker compose down` in the repo root removes it if you want it fully gone between runs.

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
- **`mserve_base` is the future command arbiter — still not built.** Nav2 and `mserve_joystick` both already publish directly to `/cmd_vel` (same topic, no priority/mutex between them) — whichever published most recently wins at any instant. `mserve_joystick` only publishes while genuinely active (see its own README) specifically to reduce accidental interference, but that's a mitigation, not real arbitration. Source arbitration and the e-stop gate are still meant to live in `mserve_base` eventually; `mserve_drivechain` always sees a single safe Twist either way, just not necessarily the one you'd expect if two sources are driving at once.
- **Parameter bounds enforced by ROS descriptors.** Out-of-range values are rejected before callbacks are called. No manual throws needed.
- **Named QoS profiles from `mserve_utils`.** `mserve_qos::commands`, `mserve_qos::feedback`, `mserve_qos::status` — profiles readable from params without recompiling.
- **No `ros2_control` yet.** The first control stack is written by hand so every part (clamping, kinematics, serial protocol, odometry, fail-safe) is visible and learnable. `ros2_control` revisited later as a comparison.
- **Hardware nodes wrap the vendor's device/driver class directly, not the vendor's ROS node.** `mserve_drivechain` wraps its own `DriveUart`, `mserve_camera` wraps `v4l2_camera`'s `V4l2CameraDevice`, `mserve_lidar` wraps a vendored RPLIDAR SDK's `sl::ILidarDriver` — never someone else's whole `rclcpp::Node`. Reasoning is the same each time: an upstream driver node is a plain node, not a lifecycle node, so it can't be configured/activated in step with the rest of the stack or driven by `lifecycle_manager`.

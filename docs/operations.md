# mServe operations reference

The complete operational reference for running, building, and debugging the
stack. The root [readme.md](../readme.md) has the short version — this doc is
the long one. Content here used to live in the root readme and is kept in
full.

## Runtime layout

Everything runs in **Docker** (image `mserve-robot:jazzy`, ROS 2 Jazzy) on
fresh Raspberry Pi OS — there is no native ROS install on this Pi at all.
`run_stack.sh` auto-detects this (`command -v ros2` fails) and routes every
command through `docker compose exec robot-mserve`.

Drivechain, base, camera (`mserve_camera`), lidar (`mserve_lidar`), rosbridge,
Foxglove Bridge (on by default, `--no-foxglove` to skip it), SLAM Toolbox
(`--slam-map`/`--slam-local`), Nav2 (`--nav2`, implies `--slam-local` if no
SLAM flag is given), and the full web UI (`drivechain.html`/`base.html`/
`camera.html`/`lidar.html`/`joystick.html`/`sensehat.html`) all run in
Docker. Device paths use udev `by-id` stable symlinks (see
`docker-compose.yml`), not raw `/dev/videoN`/`/dev/ttyUSBN` — those indices
aren't stable across USB resets/replugs on this Pi, confirmed the hard way.
`slam_toolbox` itself needed a small source patch (this distro's
`message_filters` changed API underneath it) — see
`ws/src/third_party/README.md`. Zenoh remote-RViz isn't ported to Docker yet
— same-LAN access is already covered by Foxglove Bridge; see `docs/TODO.md`
if you need it.

### Repo files

- `.env`: local runtime settings (gitignored).
- `Dockerfile` / `docker-compose.yml`: the active runtime (ROS 2 Jazzy). `run_stack.sh` auto-detects and uses them whenever `ros2` isn't found on the host PATH — true on this Pi OS install, which has no native ROS at all.
- `docs/`: design notes, milestones, session log, task tracker.
- `scripts/`: all helper scripts, flat and topic-named (`scripts/README.md` has the full list).
- `web/`: debug browser UI (lifecycle control + cmd_vel publisher), served by `scripts/run_stack.sh`.
- `ws/`: ROS 2 workspace (`ws/src/`), built with `colcon` inside the container.

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

## Simulation (Thor)

Gazebo Harmonic + RViz run on the NVIDIA Thor — the Pi 5 has no compute GPU
and stays hardware-only. `mserve_description` carries the Gazebo integration
directly (`ros_gz_sim`/`ros_gz_bridge` deps, `<sensor>` tags in the xacros,
`params/mserve_gazebo_bridge.yaml`, `worlds/empty.sdf`); there is no separate
sim package. On Thor:

```bash
./scripts/sim/launch_mserve_description_gazebo.sh   # Gazebo + RViz, robot spawned
./scripts/sim/launch_mserve_description_rviz.sh     # RViz-only view
./scripts/sim/stop_mserve_description_gazebo.sh     # stop cleanly
```

The long-term hardware-in-loop idea: Thor runs simulation, the Pi runs the
real control stack, both share topics over the network, so Pi control logic
can be validated in sim before touching hardware.

## Remote RViz (Zenoh)

Gazebo + RViz run on the NVIDIA Thor, not the Pi. For same-LAN
viewing/teleop without any of this, Foxglove Bridge (on by default, above)
is the simpler path — Zenoh is only needed for discovery across Tailscale
or other multi-interface setups where DDS/plain multicast discovery doesn't
reliably work.

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

`mserve-drivechain.service` (`Requires=docker.service` — checked into the repo at `systemd/mserve-drivechain.service`, install with `sudo cp systemd/mserve-drivechain.service /etc/systemd/system/ && sudo systemctl daemon-reload`) runs `./scripts/run_stack.sh` on boot, which starts the Docker container and builds/launches the workspace inside it, so the robot comes up ready on power-on. A `boot_splash.py` `ExecStartPre` paints an initial state (closed eyes + IP) to `/dev/fb0` before the container is even up.

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
build commands for both: **[`ws/src/third_party/README.md`](../ws/src/third_party/README.md)**
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

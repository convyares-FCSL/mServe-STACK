# mServe 🍪🤖

**A homebuilt robot whose destiny is to deliver food on request.**

mServe is the third robot in a family line. It started with **mBot** — the
little kit robot that arrived when my daughter was born. Then came **mBuff**,
the second build. mServe is the next step up: a real ROS 2 differential-drive
robot with hub motors, lidar, a camera, a face, and (increasingly) a mind of
its own. The end goal is simple and noble: you ask for a snack, and the robot
brings it.

It's not there yet — right now it drives, sees, maps the house, and navigates
through doorways on its own. The snack-carrying part is the roadmap.

## What it can do today

- 🕹️ **Drive** — from the browser, a game controller, or the Sense HAT's tiny joystick
- 👀 **See** — USB camera stream + RPLIDAR C1 laser scans, viewable live in the web UI or Foxglove
- 🗺️ **Map** — drive it around a room and SLAM Toolbox builds a map in real time
- 🧭 **Navigate** — give it a goal on the map and Nav2 + AMCL drive it there autonomously, doorways included
- 🙂 **Emote** — a touchscreen face whose eyes track where it's driving, plus an LED-matrix status display
- 🔌 **Boot ready** — powers on into a systemd service; walk up, open the web UI, drive

## The stack in one picture

```
  web UI  /  game controller  /  Sense HAT  /  Nav2
                      │
                      ▼  geometry_msgs/Twist on /cmd_vel
              ┌───────────────┐   clamps to speed limits, diff-drive
              │  mserve_base  │   kinematics, IMU-fused odometry
              └───────────────┘
                      │
                      ▼  per-motor RPM (interfaces/srv/Drive)
           ┌─────────────────────┐   pure motor driver:
           │  mserve_drivechain  │   JSON over UART, /dev/ttyAMA0
           └─────────────────────┘
                      │
                      ▼  JSON
        ESP32 on a Waveshare DDSM Driver HAT   (owns the raw DDSM115 protocol)
                      │
                      ▼
              DDSM115 hub motors × 2
```

Everything runs in **Docker** (ROS 2 Jazzy) on a **Raspberry Pi 5** — the Pi
has no native ROS install at all. Gazebo + RViz simulation runs on a separate
NVIDIA Thor; the Pi stays hardware-only.

The design philosophy is **learning-first**: every control boundary,
kinematic calculation, and hardware protocol is written by hand before
reaching for frameworks like `ros2_control`. Each layer should be readable on
its own. See [docs/architecture.md](docs/architecture.md).

## Hardware

- Raspberry Pi 5 (the brain — Docker + ROS 2 Jazzy)
- DDSM115 hub motors × 2, via a Waveshare DDSM Driver HAT — its onboard ESP32 speaks JSON over UART on the Pi's GPIO header and handles the raw motor protocol itself (firmware in `ws/src/mserve_drivechain/drive_firmware/`)
- Generic USB UVC webcam — driven by `mserve_camera`
- SLAMTEC RPLIDAR C1 (USB serial, 460800 baud) — driven by `mserve_lidar`
- ELEGOO 3.5" SPI touchscreen (ILI9486 + ADS7846 touch) — the face and touch menu, driven by `mserve_display`
- Pi Sense HAT: 8×8 LED matrix, 5-way joystick, LSM9DS1 IMU — driven by `mserve_sensehat` (the IMU feeds the odometry fusion)

## Quick start

On the robot itself the stack starts on boot (systemd) — just open the web UI:

```
http://<pi-ip>:6240/drivechain.html     (also base / camera / lidar / joystick / sensehat .html)
```

Click **Connect** on the drivechain page, then drive. To run manually instead:

```bash
./scripts/run_stack.sh              # real hardware
./scripts/run_stack.sh --sim        # simulated backend, no hardware needed
./scripts/run_stack.sh --slam-map   # + SLAM Toolbox, build a map while you drive
./scripts/run_stack.sh --nav2       # + Nav2: localize on a saved map, accept goals
```

`run_stack.sh` handles everything — starting the Docker container, building
the workspace inside it, launching all nodes, and serving the web UI. Foxglove
Bridge is on by default (`ws://<pi-ip>:8765`) for live visualization.

The full operational reference — every flag, the complete map-then-navigate
workflow, sending Nav2 goals from Foxglove, systemd management, build
internals, log locations — lives in **[docs/operations.md](docs/operations.md)**.

## Workspace tour

```
ws/src/
  interfaces/            messages, services, central YAML config
  utils/                 shared C++ helpers: params, QoS profiles, topic names
  mserve_base/           safety clamp + diff-drive kinematics + odometry (IMU-fused)
  mserve_drivechain/     pure motor driver — JSON/UART link to the ESP32
  mserve_camera/         lifecycle node, USB webcam
  mserve_lidar/          lifecycle node, RPLIDAR C1 (vendored SDK)
  mserve_display/        the face: ELEGOO touchscreen UI
  mserve_joystick/       game controller teleop (/joy → /cmd_vel)
  mserve_sensehat/       Sense HAT: LED matrix status, joystick, IMU
  mserve_description/    URDF robot model
  lifecycle_manager/     BehaviorTree.CPP-driven configure/activate/shutdown
  launch/                launch files (mserve_min.launch.py & friends)
  third_party/           vendored deps (BehaviorTree.ROS2, slam_toolbox, RTIMULib) — gitignored
```

| Package | Role |
|---------|------|
| `interfaces` | Shared messages, services, central YAML config |
| `utils` | `get_or_declare_param`, `bounded_double`, named QoS profiles, topic name utilities |
| `mserve_base` | Subscribes `/cmd_vel`, clamps to speed limits, runs diff-drive kinematics both ways (Twist → per-motor RPM, wheel feedback + IMU → `/odom`), commands the drivechain |
| `mserve_drivechain` | Pure motor driver — owns the JSON-over-UART link to the ESP32; per-motor RPM in, velocity/position feedback out |
| `mserve_camera` | Wraps `v4l2_camera`'s device class directly, publishes `Image` + `CameraInfo` — see its [README](ws/src/mserve_camera/README.md) |
| `mserve_lidar` | Wraps a vendored Slamtec SDK directly, publishes `LaserScan` — see its [README](ws/src/mserve_lidar/README.md) |
| `mserve_display` | The face — eyes track cmd_vel — plus a connect/info/calibrate touch menu — see its [README](ws/src/mserve_display/README.md) |
| `mserve_joystick` | Game controller teleop: `/joy` → `/cmd_vel` + buttons for connect, display info, speed scaling — see its [README](ws/src/mserve_joystick/README.md) |
| `mserve_sensehat` | LED matrix shows drivechain status, joystick center-click connects, publishes `sensor_msgs/Imu` — see its [README](ws/src/mserve_sensehat/README.md) |
| `mserve_description` | URDF robot description |
| `lifecycle_manager` | BT-driven configure/activate on bringup, deactivate/shutdown on exit, for the four lifecycle nodes (display/joystick/sensehat are plain nodes) — see its [README](ws/src/lifecycle_manager/README.md) |
| `launch` | `mserve_min.launch.py` starts the whole crew together |

## Design decisions

- **`mserve_drivechain` does not own kinematics.** Wheel geometry (`wheel_separation`, `wheel_radius`) and diff-drive math (both directions — `/cmd_vel` → wheel RPM, and wheel velocity feedback → odometry) live in `mserve_base`. `mserve_drivechain` is a pure motor driver — it only ever sees per-motor RPM commands and reports per-motor velocity/position feedback, so swapping the drivetrain hardware only requires changes in one package.
- **`mserve_base` is the future command arbiter — still not built.** Nav2 and `mserve_joystick` both already publish directly to `/cmd_vel` (same topic, no priority/mutex between them) — whichever published most recently wins at any instant. `mserve_joystick` only publishes while genuinely active (see its own README) specifically to reduce accidental interference, but that's a mitigation, not real arbitration. Source arbitration and the e-stop gate are still meant to live in `mserve_base` eventually; `mserve_drivechain` always sees a single clamped command stream either way, just not necessarily from the source you'd expect if two are driving at once.
- **Parameter bounds enforced by ROS descriptors.** Out-of-range values are rejected before callbacks are called. No manual throws needed.
- **Named QoS profiles from `utils`.** `mserve_qos::commands`, `mserve_qos::feedback`, `mserve_qos::status` — profiles readable from params without recompiling.
- **No `ros2_control` yet.** The first control stack is written by hand so every part (clamping, kinematics, serial protocol, odometry, fail-safe) is visible and learnable. `ros2_control` revisited later as a comparison.
- **Hardware nodes wrap the vendor's device/driver class directly, not the vendor's ROS node.** `mserve_drivechain` wraps its own `DriveUart`, `mserve_camera` wraps `v4l2_camera`'s `V4l2CameraDevice`, `mserve_lidar` wraps a vendored RPLIDAR SDK's `sl::ILidarDriver` — never someone else's whole `rclcpp::Node`. Reasoning is the same each time: an upstream driver node is a plain node, not a lifecycle node, so it can't be configured/activated in step with the rest of the stack or driven by `lifecycle_manager`.

## The road to snack delivery

Roughly in order:

1. ~~Drive by hand~~ ✅ (web, controller, Sense HAT)
2. ~~See — camera + lidar~~ ✅
3. ~~Map the house~~ ✅ (SLAM Toolbox)
4. ~~Go somewhere on its own~~ ✅ (Nav2, doorway-tuned costmaps)
5. Command arbitration + e-stop gate in `mserve_base`
6. Named destinations ("go to the kitchen")
7. Something to actually carry the food — a tray at minimum, an arm if we're feeling brave
8. "mServe, bring me a biscuit" 🍪

## Docs

- **[docs/operations.md](docs/operations.md)** — the full run/build/debug reference (start here for anything operational)
- [docs/architecture.md](docs/architecture.md) — control philosophy, ESP32 boundary, lifecycle rules, C++ conventions
- [docs/TODO.md](docs/TODO.md) — task tracker
- Per-package READMEs under `ws/src/*/README.md` — the source of truth for each package

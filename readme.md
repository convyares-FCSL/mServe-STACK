# mServe Stack

ROS 2 C++ robot stack for the mServe differential-drive robot, running natively (no Docker) on a Raspberry Pi 5. Originally built against ROS 2 Jazzy; as of the July 2026 SD-card migration it builds and runs natively against ROS 2 Lyrical — see the CMake note in [Build](#build) if you're building from scratch on a newer distro.

The drivechain + base stack starts automatically on boot via systemd (`mserve-drivechain.service`), so the robot is ready as soon as the Pi powers on.

The design philosophy is learning-first: every control boundary, kinematic calculation, and hardware protocol is written by hand before reaching for frameworks like `ros2_control`. Each layer should be readable on its own.

## Hardware

- Raspberry Pi 5 (runtime)
- DDSM115 hub motors × 2, driven via a Waveshare DDSM Driver HAT — its onboard ESP32 speaks JSON over UART on the Pi 5 GPIO header (`/dev/ttyAMA0`) and handles the raw DDSM115 protocol itself; the ESP32 firmware lives in `ws/src/mserve_drivechain/drive_firmware/`
- Gazebo + RViz simulation run on the NVIDIA Thor — the Pi 5 has no compute GPU and stays hardware-only

## Workspace

```
ws/src/
  interfaces/            central config, messages, services
  utils/                 shared utilities: params, QoS profiles, topic names
  mserve_base/           command arbiter and safety clamp
  mserve_drivechain/     diff-drive kinematics + JSON/UART protocol to the ESP32 motor controller
  mserve_description/    URDF robot model
  launch/                launch files
```

(Folder names dropped the `mserve_` prefix on `interfaces`/`utils`/`launch` at some point — the docs below still use the old prefixed names in places; treat the folder names above as current.)

### Package responsibilities

| Package | Role |
|---------|------|
| `interfaces` | Shared messages, services, central YAML config |
| `utils` | `get_or_declare_param`, `bounded_double`, named QoS profiles, topic name utilities |
| `mserve_base` | Subscribes `/cmd_vel`, clamps to speed limits, publishes `/mserve/cmd_vel_safe` |
| `mserve_drivechain` | Subscribes `/mserve/cmd_vel_safe`, runs diff-drive kinematics, owns the JSON-over-UART link to the onboard ESP32 (Waveshare DDSM Driver HAT) — the ESP32 itself handles the raw DDSM115 protocol |
| `mserve_description` | URDF robot description |
| `launch` | `mserve_min.launch.py` — starts base + drivechain |

## Files

- `.env`: local runtime settings (gitignored).
- `Dockerfile` / `docker-compose.yml`: legacy — the stack ran in Docker before the July 2026 migration to the Ubuntu 26.04 SD card. Left in place for reference but unused; `run_drivechain_hw.sh` only falls back to them if it can't find `ros2` on PATH.
- `docs/`: design notes, milestones, session log, task tracker.
- `scripts/`: helper scripts by phase.
- `web/`: debug browser UI (lifecycle control + cmd_vel publisher) and `run_drivechain_hw.sh`, the main entry point for the hardware stack.
- `ws/`: ROS 2 workspace (`ws/src/`), built natively with `colcon`.

## Docs

- `docs/plan.md` — short project plan and open questions.
- `docs/architecture.md` — control philosophy, ESP32 boundary, lifecycle rules.
- `docs/packages.md` — package-by-package design notes.
- `docs/milestones.md` — staged build path (milestones 0–4 complete).
- `docs/TODO.md` — task tracker.
- `docs/session.md` — session decisions and notes.
- `docs/testing-and-scripts.md` — test strategy and script reference.

## Normal flow

The drive stack (rosbridge + `mserve_drivechain` + `mserve_base` + web UI) starts automatically on boot via `mserve-drivechain.service` (see [Running on boot](#running-on-boot-systemd)) — **on a running Pi you normally don't need to start anything by hand, just open the web UI directly:**

- `http://<pi-ip>:6240/drivechain.html`
- `http://<pi-ip>:6240/base.html`

e.g. on this Pi (static Wi-Fi IP): `http://172.16.68.73:6240/drivechain.html` /
`http://172.16.68.73:6240/base.html` — also reachable over Tailscale at
`http://100.122.150.74:6240/...` from off-network.

To run it manually instead — e.g. after stopping the service, or for development — use:

```bash
./web/run_drivechain_hw.sh              # hardware, /dev/ttyAMA0 (Pi 5 GPIO UART)
./web/run_drivechain_hw.sh --sim        # simulated backend, no hardware needed
./web/run_drivechain_hw.sh /dev/ttyACM0 # hardware, custom UART device (e.g. USB)
```

This builds the workspace natively (falls back to the legacy `robot-mserve` Docker container only if `ros2` isn't found on PATH), starts rosbridge, configures + activates both lifecycle nodes, and serves the debug UI at the same URLs above.

The UI shows lifecycle state for both nodes and allows configure/activate/deactivate/shutdown and cmd_vel publishing. Click **Connect** before driving — activation alone doesn't open the UART port, `~/connect` does. Press Ctrl+C to stop everything if running manually — this deactivates both nodes and tears down rosbridge/web server cleanly.

## Running on boot (systemd)

`mserve-drivechain.service` (native — no Docker dependency) sources ROS 2 Lyrical + the workspace and runs `./web/run_drivechain_hw.sh` on boot, so the robot comes up ready on power-on.

```bash
sudo systemctl status mserve-drivechain      # check it's running
sudo journalctl -u mserve-drivechain -f      # live logs
sudo systemctl restart mserve-drivechain     # restart the stack
sudo systemctl stop mserve-drivechain        # stop everything (runs cleanup)
```

`stop`/`restart` send SIGTERM to the script, triggering the same cleanup as Ctrl+C.

## Build

From `/home/ecm/mServe-STACK/ws`, natively (no Docker):

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-select interfaces utils mserve_drivechain mserve_base \
  --cmake-args -DBUILD_TESTING=OFF --symlink-install
```

**CMake note (Jazzy → Lyrical):** `ament_target_dependencies()` was removed
entirely in ROS 2 Lyrical (not just deprecated — the macro doesn't exist).
The CMakeLists for `utils`, `mserve_drivechain`, and `mserve_base` were
updated to use `target_link_libraries()` with modern imported targets
instead (`rclcpp::rclcpp`, `interfaces::interfaces`, etc. — message packages
export a `<pkg>::<pkg>` aggregate target automatically via
`rosidl_cmake_aggregate_target-extras.cmake`, no extra `find_package` needed).
If you're building this from scratch on an even newer distro and hit
`Unknown CMake command "ament_target_dependencies"`, this is why.

The Docker path (`docker compose build robot-mserve`,
`scripts/05_utils/docker_build_workspace.sh`) still exists as a fallback —
`run_drivechain_hw.sh` uses it automatically if `ros2` isn't on PATH — but
is no longer the primary workflow on this Pi.

## Runtime parameter updates (no restart needed)

```bash
# Speed limits hot-swap on both nodes
ros2 param set /mserve_base limits.max_linear_speed 0.5
ros2 param set /mserve_drivechain feedback_rate 10.0

# Describe a parameter to see its valid range
ros2 param describe /mserve_base limits.max_linear_speed
```

## Stop / remove

To stop the drive stack, use `sudo systemctl stop mserve-drivechain` (or Ctrl+C if running `run_drivechain_hw.sh` manually) — see [Running on boot](#running-on-boot-systemd). Since everything runs as native processes now, `stop` is the whole story — there's no container to separately remove.

## Logs

```bash
sudo journalctl -u mserve-drivechain -f
```

rosbridge's own log is written to `/tmp/rosbridge.log` (see `run_drivechain_hw.sh`).

Grafana/Loki (if running):

```logql
{robot="mserve"}
```

## Design decisions

- **`mserve_base` does not own kinematics.** Wheel geometry (`wheel_separation`, `wheel_radius`) and diff-drive math live in `mserve_drivechain`. Swapping the drivetrain hardware only requires changes in one package.
- **`mserve_base` is the future command arbiter.** When Nav2 and manual joystick are added, source arbitration and the e-stop gate live here. `mserve_drivechain` always sees a single safe Twist.
- **Parameter bounds enforced by ROS descriptors.** Out-of-range values are rejected before callbacks are called. No manual throws needed.
- **Named QoS profiles from `mserve_utils`.** `mserve_qos::commands`, `mserve_qos::feedback`, `mserve_qos::status` — profiles readable from params without recompiling.
- **No `ros2_control` yet.** The first control stack is written by hand so every part (clamping, kinematics, serial protocol, odometry, fail-safe) is visible and learnable. `ros2_control` revisited later as a comparison.

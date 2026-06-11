# mServe Stack

ROS 2 Jazzy C++ robot stack for the mServe differential-drive robot, running on a Raspberry Pi 5 inside Docker.

The drivechain + base stack starts automatically on boot via systemd (`mserve-drivechain.service`), so the robot is ready as soon as the Pi powers on.

The design philosophy is learning-first: every control boundary, kinematic calculation, and hardware protocol is written by hand before reaching for frameworks like `ros2_control`. Each layer should be readable on its own.

## Hardware

- Raspberry Pi 5 (runtime)
- DDSM115 hub motors × 2 via DDSM Hub Motor Driver Board
- Gazebo simulation runs on a PC (WSL2) — the Pi 5 has no compute GPU

## Workspace

```
ws/src/
  mserve_interfaces/    central config, messages, services
  mserve_utils/         shared utilities: params, QoS profiles, topic names
  mserve_base/          command arbiter and safety clamp
  mserve_drivechain/    diff-drive kinematics + DDSM115 protocol
  mserve_description/   URDF robot model
  mserve_bringup/       launch files
```

### Package responsibilities

| Package | Role |
|---------|------|
| `mserve_interfaces` | Shared messages, services, central YAML config |
| `mserve_utils` | `get_or_declare_param`, `bounded_double`, named QoS profiles, topic name utilities |
| `mserve_base` | Subscribes `/cmd_vel`, clamps to speed limits, publishes `/mserve/cmd_vel_safe` |
| `mserve_drivechain` | Subscribes `/mserve/cmd_vel_safe`, runs diff-drive kinematics, owns DDSM115 protocol |
| `mserve_description` | URDF robot description |
| `mserve_bringup` | `mserve_min.launch.py` — starts base + drivechain |

## Files

- `.env`: local runtime settings (gitignored).
- `docker-compose.yml`: ROS container with ports 5002, 8080, 9090.
- `docs/`: design notes, milestones, session log, task tracker.
- `scripts/`: helper scripts by phase.
- `web/`: debug browser UI (lifecycle control + cmd_vel publisher) and `run_drivechain_hw.sh`, the main entry point for the hardware stack.
- `ws/`: ROS 2 workspace, mounted at `/ws` in the container.

## Docs

- `docs/plan.md` — short project plan and open questions.
- `docs/architecture.md` — control philosophy, ESP32 boundary, lifecycle rules.
- `docs/packages.md` — package-by-package design notes.
- `docs/milestones.md` — staged build path (milestones 0–4 complete).
- `docs/TODO.md` — task tracker.
- `docs/session.md` — session decisions and notes.
- `docs/testing-and-scripts.md` — test strategy and script reference.

## Normal flow

The drive stack (rosbridge + `mserve_drivechain` + `mserve_base` + web UI) starts automatically on boot (see [Running on boot](#running-on-boot-systemd)). To run it manually — e.g. after a reboot of the script itself, or for development — use:

```bash
./web/run_drivechain_hw.sh              # hardware, /dev/ttyAMA0 (Pi 5 GPIO UART)
./web/run_drivechain_hw.sh --sim        # simulated backend, no hardware needed
./web/run_drivechain_hw.sh /dev/ttyACM0 # hardware, custom UART device (e.g. USB)
```

This builds the workspace (native if ROS 2 is installed, otherwise inside the `robot-mserve` Docker container), starts rosbridge, configures + activates both lifecycle nodes, and serves the debug UI. Then open:

- `http://<pi-ip>:6240/drivechain.html`
- `http://<pi-ip>:6240/base.html`

The UI shows lifecycle state for both nodes and allows configure/activate/deactivate/shutdown and cmd_vel publishing. Press Ctrl+C to stop everything — this deactivates both nodes and tears down rosbridge/web server cleanly.

## Running on boot (systemd)

`mserve-drivechain.service` runs `./web/run_drivechain_hw.sh` automatically once Docker is up, so the robot comes up ready on power-on.

```bash
sudo systemctl status mserve-drivechain      # check it's running
sudo journalctl -u mserve-drivechain -f      # live logs
sudo systemctl restart mserve-drivechain     # restart the stack
sudo systemctl stop mserve-drivechain        # stop everything (runs cleanup)
```

`stop`/`restart` send SIGTERM to the script, triggering the same cleanup as Ctrl+C.

## Build

From `/home/ecm/mServe-STACK`:

```bash
docker compose build robot-mserve
scripts/05_utils/docker_build_workspace.sh
```

Raw Docker command:

```bash
docker compose exec robot-mserve bash -lc "
  source /opt/ros/jazzy/setup.bash && cd /ws &&
  colcon build --symlink-install \
    --packages-select \
      mserve_interfaces \
      mserve_utils \
      mserve_base \
      mserve_drivechain \
      mserve_description \
      mserve_bringup
"
```

## Runtime parameter updates (no restart needed)

```bash
# Speed limits hot-swap on both nodes
ros2 param set /mserve_base limits.max_linear_speed 0.5
ros2 param set /mserve_drivechain feedback_rate 10.0

# Describe a parameter to see its valid range
ros2 param describe /mserve_base limits.max_linear_speed
```

## Stop / remove

To stop the drive stack, use `sudo systemctl stop mserve-drivechain` (or Ctrl+C if running `run_drivechain_hw.sh` manually) — see [Running on boot](#running-on-boot-systemd).

The commands below stop/remove the underlying `robot-mserve` container itself. Don't run `docker compose down` while `mserve-drivechain.service` is enabled — it brings the container back up (`docker compose up -d robot-mserve`) on every start/restart.

```bash
docker compose stop robot-mserve   # stop, keep container
docker compose down                 # remove container
```

## Logs

```bash
docker compose logs -f robot-mserve
```

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

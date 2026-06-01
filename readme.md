# mServe Stack

ROS 2 Jazzy C++ robot stack for the mServe differential-drive robot, running on a Raspberry Pi 5 inside Docker.

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
- `web/`: debug browser UI (lifecycle control + cmd_vel publisher).
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

```bash
# 1. Build and start the container image
docker compose up -d --build robot-mserve

# 2. Build the ROS workspace inside Docker
scripts/05_utils/docker_build_workspace.sh

# 3. Launch ROS nodes (base + drivechain)
scripts/05_utils/docker_launch_mserve.sh

# 4. Start rosbridge and the web UI
scripts/05_utils/docker_webbridge.sh both
```

Then open `http://localhost:8080` — the UI shows lifecycle state for both nodes and allows configure/activate/deactivate/shutdown and cmd_vel publishing.

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

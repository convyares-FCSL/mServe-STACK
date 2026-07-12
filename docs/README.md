# mServe Docs

All project documentation should live in this folder unless it is a short root-level entry point like `readme.md` or `plan.md`.

## Files

- `architecture.md`: design philosophy and technical boundaries.
- `packages.md`: planned ROS 2 package skeletons.
- `milestones.md`: staged build plan.
- `testing-and-scripts.md`: script and test strategy.
- `simulation_hil.md`: current simulation status, next steps, and hardware-in-loop plan.
- `remote-rviz-setup.md`: guide to running RViz on a development laptop connected to ROS/Gazebo on a remote machine (e.g., NVIDIA Thor).
- `TODO.md`: task tracker.
- `session.md`: session notes, decisions, and what changed.

## Source Guide

The C++ lessons in `/home/ecm/ros2-systems-operability/src/2_cpp` are the main style guide:

- Lesson 05: central YAML configuration.
- Lesson 06: lifecycle nodes.
- Lesson 07: actions.
- Lesson 08: callback groups and executors.
- Lesson 09: composition.
- Lesson 10: launch topology and deployment verification.

## Running the Current Skeleton

**As of the July 2026 SD-card migration, this Pi runs the stack natively — no
Docker.** The Docker workflow below is kept as a documented fallback only
(`run_stack.sh` uses it automatically if `ros2` isn't found on PATH);
it is no longer the recommended path on this machine.

### Host-native workflow (recommended)

From `/home/ecm/mServe-STACK`:

```bash
source /opt/ros/lyrical/setup.bash
cd ws
colcon build --symlink-install --packages-select interfaces utils mserve_base mserve_drivechain mserve_description
source install/setup.bash
ros2 launch launch mserve_min.launch.py
```

Or use the one-command launcher, which builds, starts rosbridge, activates
both lifecycle nodes, and serves the debug web UI:

```bash
cd /home/ecm/mServe-STACK
./scripts/run_stack.sh          # hardware
./scripts/run_stack.sh --sim    # sim backend, no hardware needed
```

**Note on ROS distro:** this was originally built against ROS 2 Jazzy; it now
builds natively against ROS 2 Lyrical. `ament_target_dependencies()` was
removed in Lyrical — see the CMake note in the root `readme.md` Build section
if you're building from scratch on a newer distro.

### Docker workflow (legacy fallback)

```bash
docker compose up -d --build robot-mserve
scripts/docker/docker_build_workspace.sh
scripts/docker/docker_launch_mserve.sh
scripts/docker/docker_webbridge.sh both
```

This launches rosbridge inside the `robot-mserve` container and serves the web UI at `http://localhost:8080`.

If you want to run the raw command instead of the helper, use:

```bash
docker compose exec robot-mserve bash -lc "cd /ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch launch mserve_min.launch.py"
```

This current skeleton provides:

- `mserve_base`: the robot drive lifecycle node that accepts `/cmd_vel`.
- `mserve_drivechain`: diff-drive kinematics + the JSON/UART link to the onboard
  ESP32 motor controller (Waveshare DDSM Driver HAT), which drives the DDSM115
  hub motors. Not a stub — this is the real hardware boundary. See
  `ws/src/mserve_drivechain/README.md` for the full protocol writeup.
- `launch`: the central bringup launch package (was planned as `mserve_bringup`;
  the actual folder/package name is `launch`).
- `interfaces`: shared ROS messages, services, actions, and config (was planned
  as `mserve_interfaces`).
- `utils`: shared C++ helper support (was planned as `mserve_utils`).

## Development Web Bridge

A simple browser UI is available under `web/` for lifecycle and drive command testing.

The current entry point is `./scripts/run_stack.sh` (see above), which
serves the UI on port **6240** — `http://<pi-ip>:6240/drivechain.html`,
`.../base.html`, `.../camera.html`. `scripts/run_rosbridge.sh` and
`scripts/run_web_only.sh` are standalone helpers for when the drive stack is
already running some other way (see `web/README.md`).

The UI connects to ROS via rosbridge at `ws://localhost:9090` and can:

- query `/mserve_base`, `/mserve_drivechain`, `/mserve_camera` lifecycle state
- trigger lifecycle transitions
- publish `/cmd_vel` commands

`lifecycle_manager` now drives configure/activate/shutdown on bringup — this
web bridge is for manual transitions and observability during development,
not the primary bringup path anymore.

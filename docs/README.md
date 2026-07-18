# mServe Docs

All project documentation should live in this folder unless it is a short root-level entry point like `readme.md` or `plan.md`.

## Files

- `architecture.md`: design philosophy and technical boundaries.
- `packages.md`: planned ROS 2 package skeletons.
- `milestones.md`: staged build plan.
- `testing-and-scripts.md`: script and test strategy.
- `simulation_hil.md`: current simulation status, next steps, and hardware-in-loop plan.
- `remote-rviz-zenoh.md`: current guide to running RViz on a remote machine (e.g., NVIDIA Thor) connected to the Pi over a Zenoh router.
- `remote-rviz-setup.md`: superseded — the earlier Fast DDS discovery server + WSL approach, kept for reference.
- `TODO.md`: task tracker.
- `session.md`: session notes, decisions, and what changed.
- `camera/todo.md`, `lidar/todo.md`, `lifecycle_manager/{todo,lesson_plan}.md`:
  per-package known-limitations/notes docs. Live here, not alongside their
  package source under `ws/src/` — keeps all project docs in one place
  rather than spread throughout the repo. Each package's own `README.md`
  points here.

## Source Guide

The C++ lessons in `/home/ecm/ros2-systems-operability/src/2_cpp` are the main style guide:

- Lesson 05: central YAML configuration.
- Lesson 06: lifecycle nodes.
- Lesson 07: actions.
- Lesson 08: callback groups and executors.
- Lesson 09: composition.
- Lesson 10: launch topology and deployment verification.

## Running the Current Skeleton

**This Pi runs the stack in Docker (ROS 2 Jazzy) — there is no native ROS
install on this Pi at all.** `run_stack.sh` detects this (`command -v ros2`
fails) and routes every command through `docker compose exec robot-mserve`.

### Docker workflow (the only path on this Pi)

The one-command launcher builds, starts rosbridge, activates the lifecycle
nodes, and serves the debug web UI, all inside the container:

```bash
cd /home/ecm/mServe-STACK
./scripts/run_stack.sh          # hardware
./scripts/run_stack.sh --sim    # sim backend, no hardware needed
```

If you want to run the raw command instead, from inside the container
(`docker compose exec robot-mserve bash`):

```bash
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --symlink-install --packages-select interfaces utils mserve_base mserve_drivechain mserve_camera mserve_lidar mserve_display mserve_description lifecycle_manager launch btcpp_ros2_interfaces behaviortree_ros2
source install/setup.bash
ros2 launch launch mserve_min.launch.py
```

`scripts/docker/docker_build_workspace.sh`, `docker_launch_mserve.sh`, and
`docker_webbridge.sh` still exist on disk but are superseded by
`run_stack.sh`'s own built-in Docker orchestration above — no need to run
them separately.

If you're building on a machine with ROS 2 installed natively instead of in
Docker, the same `colcon build`/`ros2 launch` commands above work unchanged
from that machine's own workspace root.

This current skeleton provides:

- `mserve_base`: the robot drive lifecycle node that accepts `/cmd_vel`.
- `mserve_drivechain`: diff-drive kinematics + the JSON/UART link to the onboard
  ESP32 motor controller (Waveshare DDSM Driver HAT), which drives the DDSM115
  hub motors. Not a stub — this is the real hardware boundary. See
  `ws/src/mserve_drivechain/README.md` for the full protocol writeup.
- `mserve_camera`: lifecycle node wrapping `v4l2_camera`'s device class directly
  (USB UVC webcam), publishing `sensor_msgs/Image` + `CameraInfo`. See
  `ws/src/mserve_camera/README.md`.
- `mserve_lidar`: lifecycle node wrapping a vendored Slamtec RPLIDAR SDK
  directly (RPLIDAR C1), publishing `sensor_msgs/LaserScan`. See
  `ws/src/mserve_lidar/README.md`.
- `mserve_display`: plain node (not lifecycle-managed) driving the ELEGOO
  3.5" SPI touchscreen — face w/ eyes tracking `cmd_vel`, connect/info/
  calibrate menu. See `ws/src/mserve_display/README.md`.
- `launch`: the central bringup launch package (was planned as `mserve_bringup`;
  the actual folder/package name is `launch`).
- `interfaces`: shared ROS messages, services, actions, and config (was planned
  as `mserve_interfaces`).
- `utils`: shared C++ helper support (was planned as `mserve_utils`).

## Development Web Bridge

A simple browser UI is available under `web/` for lifecycle and drive command testing.

The current entry point is `./scripts/run_stack.sh` (see above), which
serves the UI on port **6240** — `http://<pi-ip>:6240/drivechain.html`,
`.../base.html`, `.../camera.html`, `.../lidar.html`. `scripts/run_rosbridge.sh`
and `scripts/run_web_only.sh` are standalone helpers for when the drive stack
is already running some other way (see `web/README.md`).

The UI connects to ROS via rosbridge at `ws://localhost:9090` and can:

- query `/mserve_base`, `/mserve_drivechain`, `/mserve_camera`, `/mserve_lidar` lifecycle state
- trigger lifecycle transitions
- publish `/cmd_vel` commands

`lifecycle_manager` now drives configure/activate/shutdown on bringup — this
web bridge is for manual transitions and observability during development,
not the primary bringup path anymore.

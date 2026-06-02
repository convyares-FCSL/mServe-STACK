# mServe Docs

All project documentation should live in this folder unless it is a short root-level entry point like `readme.md` or `plan.md`.

## Files

- `architecture.md`: design philosophy and technical boundaries.
- `packages.md`: planned ROS 2 package skeletons.
- `milestones.md`: staged build plan.
- `testing-and-scripts.md`: script and test strategy.
- `simulation_hil.md`: current simulation status, next steps, and hardware-in-loop plan.
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

### Docker-based workflow (recommended)

From `/home/ecm/mServe-STACK`:

```bash
docker compose up -d --build robot-mserve
```

Build and run inside the container:

```bash
scripts/05_utils/docker_build_workspace.sh
```

Launch the bringup using the Docker helper:

```bash
cd /home/ecm/mServe-STACK
scripts/05_utils/docker_launch_mserve.sh
```

Start the web bridge using Docker:

```bash
cd /home/ecm/mServe-STACK
scripts/05_utils/docker_webbridge.sh both
```

This launches rosbridge inside the `robot-mserve` container and serves the web UI at `http://localhost:8080`.

If you want to run the raw command instead of the helper, use:

```bash
docker compose exec robot-mserve bash -lc "cd /ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export AMENT_PREFIX_PATH=/ws/install/mserve_bringup:/ws/install/mserve_description:/ws/install/mserve_base:/ws/install/mserve_drivechain:/ws/install/mserve_utils:/ws/install/mserve_interfaces:\$AMENT_PREFIX_PATH && ros2 launch mserve_bringup mserve_min.launch.py"
```

Check lifecycle state from within the same container:

```bash
docker compose exec robot-mserve bash -lc "cd /ws && source install/setup.bash && ros2 lifecycle get /mserve_base"

docker compose exec robot-mserve bash -lc "cd /ws && source install/setup.bash && ros2 lifecycle get /mserve_drivechain"
```

### Host-native workflow (only if ROS is installed on the host)

If you have a host ROS environment, the original workflow still applies:

```bash
./scripts/01_setup/env_setup.sh
cd ws
colcon build --symlink-install --packages-select mserve_interfaces mserve_utils mserve_base mserve_drivechain mserve_description mserve_bringup
source install/setup.bash
ros2 launch mserve_bringup mserve_min.launch.py
```

The Docker workflow is recommended when the host does not have ROS installed or when you want the same environment used by Loki/log collection.

This current skeleton provides:

- `mserve_base`: the robot drive lifecycle node that accepts `/cmd_vel`.
- `mserve_drivechain`: the drivetrain/ESP32 boundary stub that publishes feedback and status.
- `mserve_bringup`: the central bringup launch package.
- `mserve_interfaces`: shared ROS messages, services, actions, and config.
- `mserve_utils`: shared C++ helper support for later utilities.

## Development Web Bridge

A simple browser UI is available under `web/` for lifecycle and drive command testing.

To use it:

```bash
cd /home/ecm/mServe-STACK/web
./run_rosbridge.sh
./run.sh
```

Then open `http://localhost:8080`.

The UI connects to ROS via rosbridge at `ws://localhost:9090` and can:

- query `/mserve_base` and `/mserve_drivechain` lifecycle state
- trigger lifecycle transitions
- publish `/cmd_vel` commands

A central lifecycle manager is still a future milestone, but this web bridge makes the current node-level lifecycle visible and controllable during development.

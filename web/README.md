# mServe Web Bridge

This folder contains a development-only web UI for the mServe lifecycle nodes.

The web UI connects to ROS 2 through `rosbridge_server` and uses `roslibjs`.

## Goals

- display the lifecycle state of `mserve_base` and `mserve_drivechain`
- allow lifecycle transitions from the browser
- publish `/cmd_vel` commands to drive the robot from the browser

## Run

1. Start the ROS environment and build the workspace:

```bash
cd /home/ecm/mServe-STACK
./scripts/01_setup/env_setup.sh
cd ws
colcon build --symlink-install --packages-select mserve_interfaces mserve_utils mserve_base mserve_esp32 mserve_bringup
source install/setup.bash
```

2. Launch the mServe nodes:

```bash
ros2 launch mserve_bringup mserve_min.launch.py
```

3. Start rosbridge:

```bash
ros2 run rosbridge_server rosbridge_websocket --port 9090
```

4. Start the static web server:

```bash
cd /home/ecm/mServe-STACK/web
./run.sh
```

5. Open the browser:

`http://localhost:6240`

## Notes

- This web UI is development-only.
- If `rosbridge_server` is not installed, install it in your ROS 2 environment first.

# mServe Web Bridge

This folder contains a development-only web UI for the mServe lifecycle nodes.

The web UI connects to ROS 2 through `rosbridge_server` and uses `roslibjs`.

## Goals

- display the lifecycle state of `mserve_base` and `mserve_drivechain`
- allow lifecycle transitions from the browser
- publish `/cmd_vel` commands to drive the robot from the browser

## Run

The normal entry point is `run_drivechain_hw.sh`, which builds nothing itself
but expects the workspace already built (see the top-level readme's Build
section — `interfaces utils mserve_drivechain mserve_base lifecycle_manager`
plus the vendored `btcpp_ros2_interfaces`/`behaviortree_ros2`), then starts
rosbridge, launches `mserve_drivechain` + `mserve_base` + `lifecycle_manager`
via `launch/mserve_min.launch.py`, waits for both nodes to reach `active`,
and serves this folder on port 6240:

```bash
cd /home/ecm/mServe-STACK
./web/run_drivechain_hw.sh              # hardware, /dev/ttyAMA0
./web/run_drivechain_hw.sh --sim        # sim backend, no hardware needed
./web/run_drivechain_hw.sh /dev/ttyACM0 # hardware, custom UART device
```

Then open:

- `http://<pi-ip>:6240/drivechain.html`
- `http://<pi-ip>:6240/base.html`

Press Ctrl+C to stop everything — this runs `lifecycle_manager`'s shutdown
tree (deactivates both nodes) before tearing down rosbridge/web server.

`run.sh` (serves this folder only, no ROS nodes/rosbridge) and
`run_rosbridge.sh` are standalone helpers for when you already have the
drive stack running some other way — most of the time you want
`run_drivechain_hw.sh` instead.

## Notes

- This web UI is development-only.
- If `rosbridge_server` is not installed: `sudo apt install ros-lyrical-rosbridge-server`.

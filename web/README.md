# mServe Web Bridge

This folder contains a development-only web UI for the mServe lifecycle nodes
— static HTML/JS/CSS only. All launcher scripts live in `../scripts/`, not
here (see `../scripts/README.md` for the full list).

The web UI connects to ROS 2 through `rosbridge_server` and uses `roslibjs`.

## Goals

- display the lifecycle state of `mserve_base`, `mserve_drivechain`, `mserve_camera`, and `mserve_lidar`
- allow lifecycle transitions from the browser
- publish `/cmd_vel` commands to drive the robot from the browser
- preview the camera stream (`camera.html`, plus a smaller preview on `base.html`)
- plot the live `/scan` (`lidar.html`)

## Run

The normal entry point is `../scripts/run_stack.sh`, which builds nothing
itself but expects the workspace already built (see the top-level readme's
Build section — `interfaces utils mserve_drivechain mserve_base
lifecycle_manager mserve_camera mserve_lidar` plus the vendored
`btcpp_ros2_interfaces`/`behaviortree_ros2`), then starts rosbridge, launches
the full stack via `launch/mserve_min.launch.py`, waits for the lifecycle
nodes to reach `active`, and serves this folder on port 6240:

```bash
cd /home/ecm/mServe-STACK
./scripts/run_stack.sh              # hardware, /dev/ttyAMA0
./scripts/run_stack.sh --sim        # sim backend, no hardware needed
./scripts/run_stack.sh /dev/ttyACM0 # hardware, custom UART device
```

Then open:

- `http://<pi-ip>:6240/drivechain.html`
- `http://<pi-ip>:6240/base.html`
- `http://<pi-ip>:6240/camera.html`
- `http://<pi-ip>:6240/lidar.html`

Press Ctrl+C to stop everything — this runs `lifecycle_manager`'s shutdown
tree (deactivates the nodes) before tearing down rosbridge/web server.

`scripts/run_web_only.sh` (serves this folder only, no ROS nodes/rosbridge)
and `scripts/run_rosbridge.sh` are standalone helpers for when you already
have the drive stack running some other way — most of the time you want
`scripts/run_stack.sh` instead.

## Notes

- This web UI is development-only.
- If `rosbridge_server` is not installed: `sudo apt install ros-lyrical-rosbridge-server`.
- Camera preview uses `web_video_server` (MJPEG transcode) on port 8080 — see `ws/src/mserve_camera/README.md`.

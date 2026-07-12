# TODO

## Now
- [x] Scaffold `scripts/` hygiene and build helpers.
- [x] Scaffold `mserve_interfaces` with central config and first messages.
- [x] Scaffold `mserve_utils` with config/QoS validation tests.
- [x] Scaffold `mserve_base` lifecycle node.
- [x] Scaffold `mserve_drivechain` lifecycle node in dry-run mode.
- [x] Scaffold `launch` (planned as `mserve_bringup`) minimal launch.
- [x] Decide first ESP32 transport: **UART** (`/dev/ttyAMA0`, Pi 5 GPIO header), JSON protocol. Implemented in `mserve_drivechain/src/drivechain_uart.cpp`, not a separate transport-agnostic layer.
- [x] Scaffold `lifecycle_manager` (BT.CPP + `behaviortree_ros2`, vendored at `ws/src/BehaviorTree.ROS2/`) to drive `mserve_drivechain` + `mserve_base` through configure/activate on bringup and a graceful shutdown tree on SIGINT/SIGTERM. Wired into `launch/mserve_min.launch.py`; `run_drivechain_hw.sh` no longer drives lifecycle transitions itself. See `ws/src/lifecycle_manager/README.md` + `docs/todo.md`.
- [x] Remove leftover `hyfleet_subsystem`-derived scaffolding not used by mserve: `mserve_base_archive/`, and the booster/compression msgs+srvs (`ControlBooster`, `ControlCompressor`, `SystemState`, `BoosterCmd`, `CompressorCmd`, `DispenserCmd`, `GasRouterCmd`, `SetMode`, `Cmd`) from `interfaces/`.
- [x] Add `mserve_camera` package — lifecycle node wrapping `v4l2_camera`'s `V4l2CameraDevice` directly (USB UVC webcam, YUYV @ 640x480), publishing standard `sensor_msgs/Image` + `CameraInfo`. Wired into `lifecycle_manager` bringup/shutdown and `mserve_min.launch.py`. Web UI: `web/camera.html` (lifecycle/params/image/camera_info/log) via `ros-lyrical-web-video-server` (MJPEG transcode, port 8080 — raw YUYV isn't browser-decodable directly), plus a live image preview added to `web/base.html`. See `ws/src/mserve_camera/README.md` + `docs/todo.md`.
- [ ] Decide whether first robot model is plain URDF or minimal Xacro.

## Next

- [ ] Add launch tests for minimal bringup.
- [ ] Add robot description and TF smoke test.
- [ ] Add Gazebo simulation package.
- [ ] Add lidar package.
- [ ] Fix `mserve_camera` frame rate — stuck at ~12.6Hz because the V4L2 frame interval (`VIDIOC_S_PARM`) is never explicitly requested, only pixel format/resolution. See `ws/src/mserve_camera/docs/todo.md`.
- [ ] Mic (`audio_common`) — `ros-lyrical-audio-capture` depends on `libgstreamer-plugins-good1.0-0`, which doesn't exist in this Ubuntu Resolute release's repos at all. Deferred; hand-rolling an ALSA capture node is the likely path if revisited (see `ws/src/mserve_camera/docs/todo.md`).
- [ ] Add display package.
- [ ] Add Nav2 simulation launch.

## Later

- [ ] Add real ESP32 packet protocol.
- [ ] Add hardware safety behavior for ESP32 comms loss.
- [ ] Add AI package.
- [ ] Add arm/manipulation package.
- [ ] Revisit `ros2_control` later as a comparison exercise, not as the first implementation.

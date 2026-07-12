# TODO

## Now
- [x] Scaffold `scripts/` hygiene and build helpers.
- [x] Scaffold `mserve_interfaces` with central config and first messages.
- [x] Scaffold `mserve_utils` with config/QoS validation tests.
- [x] Scaffold `mserve_base` lifecycle node.
- [x] Scaffold `mserve_drivechain` lifecycle node in dry-run mode.
- [x] Scaffold `launch` (planned as `mserve_bringup`) minimal launch.
- [x] Decide first ESP32 transport: **UART** (`/dev/ttyAMA0`, Pi 5 GPIO header), JSON protocol. Implemented in `mserve_drivechain/src/drivechain_uart.cpp`, not a separate transport-agnostic layer.
- [x] Scaffold `lifecycle_manager` (BT.CPP + `behaviortree_ros2`, vendored at `ws/src/BehaviorTree.ROS2/`) to drive `mserve_drivechain` + `mserve_base` through configure/activate on bringup and a graceful shutdown tree on SIGINT/SIGTERM. Wired into `launch/mserve_min.launch.py`; `run_stack.sh` no longer drives lifecycle transitions itself. See `ws/src/lifecycle_manager/README.md` + `docs/todo.md`.
- [x] Remove leftover `hyfleet_subsystem`-derived scaffolding not used by mserve: `mserve_base_archive/`, and the booster/compression msgs+srvs (`ControlBooster`, `ControlCompressor`, `SystemState`, `BoosterCmd`, `CompressorCmd`, `DispenserCmd`, `GasRouterCmd`, `SetMode`, `Cmd`) from `interfaces/`.
- [x] Add `mserve_camera` package — lifecycle node wrapping `v4l2_camera`'s `V4l2CameraDevice` directly (USB UVC webcam, YUYV @ 640x480), publishing standard `sensor_msgs/Image` + `CameraInfo`. Wired into `lifecycle_manager` bringup/shutdown and `mserve_min.launch.py`. Web UI: `web/camera.html` (lifecycle/params/image/camera_info/log) via `ros-lyrical-web-video-server` (MJPEG transcode, port 8080 — raw YUYV isn't browser-decodable directly), plus a live image preview added to `web/base.html`. See `ws/src/mserve_camera/README.md` + `docs/todo.md`.
- [x] Add `robot_state_publisher` to `mserve_min.launch.py`, publishing `/robot_description` + TF from `mserve_description`'s URDF — needed for a remote RViz (Thor) to place `camera_link_optical` relative to `base_link`. Verified working over the LAN (and separately over a `rmw_zenoh_cpp` router across Tailscale — see `docs/session.md`).
- [x] Add real wheel odometry in `mserve_base` — `UpdateOdometry`/`PublishOdometry` BT nodes integrate `/odom` (`nav_msgs/Odometry`), broadcast `odom -> base_link` TF, and publish `/joint_states` for `left_wheel_joint`/`right_wheel_joint` (fixing the RViz TF errors on those two frames). Integrates from wheel *velocity*, not position — `mserve_drivechain`'s DDSM115 protocol never reports position in the speed-loop mode `mserve_base` actually drives in. Along the way, fixed a real bug: `mserve_drivechain` sign-corrected `velocity_rpm` for physically-reversed motors but not `position_rad`, and never populated `DriveMotorFeedback.stamp` at all. Verified in both `--sim` and on real hardware (short bounded test drive). See `ws/src/mserve_base/README.md`.
- [ ] Decide whether first robot model is plain URDF or minimal Xacro.
- [x] Reorganize all scripts into a single flat `scripts/` folder (previously split across `web/*.sh` and numbered `scripts/0N_*` phase folders). Renamed the main entry point `run_drivechain_hw.sh` → `run_stack.sh` (it now launches the whole stack — drivechain, base, camera, robot_state_publisher, lifecycle_manager — not just the drivechain). Updated `mserve-drivechain.service`'s `ExecStart`, and every doc reference across the repo. Verified end-to-end in `--sim` mode: both nodes reach `active`, web UI serves correctly, and Ctrl+C/SIGINT cleanly tears everything down with zero leftover processes. See `scripts/README.md` for the current layout.

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

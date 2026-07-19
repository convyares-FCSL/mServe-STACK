# TODO

Statements of what's next, not history — the story of how things got done
lives in `git log`.

## Next

- [ ] Command arbitration + e-stop gate in `mserve_base`. Nav2 and
  `mserve_joystick` both publish to `/cmd_vel` with no priority between them —
  last publisher wins. `mserve_base` is the designed home for source
  arbitration and an e-stop.
- [ ] Fold drivechain hardware connect/disconnect into lifecycle
  activate/deactivate — remove the separate `~/connect` service and update all
  callers (web UI, joystick, Sense HAT) to drive lifecycle transitions instead.
- [ ] Continue Nav2 tuning: AMCL odometry noise model (`alpha1`–`alpha5` still
  at Nav2 defaults), velocity caps (deliberately conservative), controller
  behavior. Doorway passage is solved; the rest is untouched.
- [ ] Named destinations ("go to the kitchen") — map map-frame poses to names,
  accept a name, send the Nav2 goal.
- [ ] Add battery monitoring (ESP32 reports it; nothing consumes it yet).
- [ ] Tie battery level into the Face screen's eyes — droop/half-close as
  `drivechain_status.battery_level` falls, fully close at a low threshold.
  `mserve_display` already subscribes to `drivechain_status`; the eyelid shapes
  exist in `scripts/boot_splash.py`. Needs render logic in
  `renderFace`/`screens.cpp`.
- [ ] Fix `mserve_camera` frame rate — stuck at ~12.6Hz vs the format table's
  30Hz. Per-fd `VIDIOC_S_PARM` is a verified no-op on this driver; a different
  mechanism is needed. See `docs/camera/todo.md`.
- [ ] Add mic capture — re-evaluate `audio_common` now that the platform is
  Docker + Jazzy (the old blocker was a missing GStreamer dependency on the
  since-reverted native Lyrical install; it may simply work now). Fallback:
  hand-rolled ALSA capture node. See `docs/camera/todo.md`.
- [ ] Add speaker output.
- [ ] Add real ESP32 packet protocol (binary framing/CRC to replace JSON once
  the protocol is stable).
- [ ] Add hardware safety behavior on the ESP32 for Pi comms loss.
- [ ] Consider switching `slam_toolbox` from vendored+patched source to the
  apt package (`ros-jazzy-slam-toolbox`). Deliberately not done: our two
  patches fix genuine upstream bugs (Boost serialization gap breaking
  `serialize_map`; a `message_filters` constructor mismatch) and it's unknown
  whether the apt build has them. If revisited, re-test `save_map` **and**
  `serialize_map` before trusting it — the Boost bug only breaks the latter.
  See `ws/src/third_party/README.md`.
- [ ] Add arm/manipulation package (`mserve_manipulation`) — next major phase
  after navigation, not in parallel. `arm_mount_link` is already a reserved
  frame in the URDF. Arm likely controlled by a separate Pi.
- [ ] Add AI package — image detection first.
- [ ] Add AI commands — track / go-to-position on top of detection + Nav2.
- [ ] Fill the unit-test gap: diff-drive math, command clamping,
  timeout/fail-safe, QoS/validation — `mserve_base/test/` is empty; only
  `utils/test/test_config.cpp` and
  `mserve_drivechain/test/test_packet_codec.cpp` exist.
- [ ] Revisit `ros2_control` as a comparison exercise, not a rewrite.
- [ ] Consider composable-node registration / process-plane split (control /
  motor comms / sensors / nav) once the standalone nodes are stable.

## Deferred

- [ ] Port Zenoh remote-RViz (`rmw_zenoh_cpp`, for RViz on the Thor across
  Tailscale) to the Docker + Jazzy setup — the `scripts/remote/` scripts
  assume native `ros2` on PATH. Likely needs `network_mode: host` in
  `docker-compose.yml`; DDS/Zenoh multicast discovery doesn't traverse
  Docker's bridge network cleanly. Same-LAN visualization is already covered
  by Foxglove Bridge, which is why this can wait.

## Done

One line each, newest first. Details: `git log` and the per-package READMEs.

- [x] Foxglove Bridge on by default (`--no-foxglove` to skip).
- [x] Nav2 doorway navigation fixed — measured-footprint polygon + retuned inflation on both costmaps (see `nav2_params.yaml` comments).
- [x] Nav2 wired up (`--nav2`, AMCL + map_server localization) and verified on real hardware; also fixed `lifecycle_manager` treating BT FAILURE as success.
- [x] SLAM map/localize modes split (`--slam-map`/`--slam-local`); four blocking bugs fixed along the way (fixed-size scans, launch param path, stale install, Boost serialization patch — see `ws/src/third_party/README.md`).
- [x] `mserve_sensehat` package — LED matrix status, joystick connect, IMU fused into odometry.
- [x] `mserve_joystick` package — game controller teleop.
- [x] `mserve_display` package — Face/Menu/Info/Calibrate on the ELEGOO touchscreen, plus boot splash.
- [x] Gazebo sim verified end-to-end on the Thor.
- [x] SLAM Toolbox brought under BT-managed lifecycle (second `lifecycle_manager` instance).
- [x] JPEG-compressed camera topic (fixed multi-second Foxglove lag over the WebSocket bridges).
- [x] Foxglove Bridge support (`scripts/run_foxglove_bridge.sh`).
- [x] Launch tests + robot-description TF smoke test (and fixed the stale `test_urdf_load.py` they surfaced).
- [x] `mserve_lidar` package (RPLIDAR C1, vendored Slamtec SDK).
- [x] Real wheel odometry in `mserve_base` (`/odom`, `odom -> base_link` TF, `/joint_states`).
- [x] `robot_state_publisher` in minimal bringup.
- [x] `mserve_camera` package (wraps `v4l2_camera`'s device class) + web UI page.
- [x] Scripts reorganized flat under `scripts/`; entry point renamed to `run_stack.sh`.
- [x] `lifecycle_manager` (BT.CPP) drives all lifecycle transitions.
- [x] Removed leftover `hyfleet_subsystem` scaffolding from `interfaces`/`mserve_base_archive`.
- [x] ESP32 transport decided and implemented: JSON over UART on `/dev/ttyAMA0`.
- [x] Initial scaffolds: `interfaces`, `utils`, `mserve_base`, `mserve_drivechain`, `launch`, remote control, Xacro robot model.

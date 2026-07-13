# mserve_camera — TODO

## Stage 1 — COMPLETE (2026-07-12)

- [x] Scaffold lifecycle node wrapping `v4l2_camera::V4l2CameraDevice`
- [x] Publish `sensor_msgs/Image` (`camera/image_raw`) + `sensor_msgs/CameraInfo` (`camera/camera_info`)
- [x] Wire into `lifecycle_manager`'s bringup/shutdown trees
- [x] Wire into `mserve_min.launch.py`
- [x] Add to `run_stack.sh`'s process-cleanup lists (drivechain/base/camera/lifecycle_manager)
- [x] Web UI: `web/camera.html` — lifecycle, parameters, live image (via `web_video_server`), camera_info, logs
- [x] Web UI: image preview added to `web/base.html` too (for driving while watching the feed)
- [x] Verified end-to-end in `--sim` and with the real USB webcam: bringup → active → frames flowing → graceful shutdown

## Known limitations / next up

- [x] **Compressed image for bandwidth-constrained subscribers** — added
  (2026-07-13) `camera/image_raw/compressed` (`sensor_msgs/CompressedImage`,
  JPEG, `jpeg_quality` param). Root cause: `foxglove_bridge`/`rosbridge`
  multiplex every topic over one WebSocket connection, and raw YUYV
  (~600KB/frame) was queuing ahead of small time-critical messages like
  `/tf` sharing that connection — observed as cmd_vel/teleop feeling instant
  while the robot model visibly lagged ~12s behind in Foxglove. See
  README.md's "Compressed image" section for why this is hand-rolled
  (`cv::cvtColor` + `cv::imencode`) rather than via `image_transport`'s
  plugin system. **Verified against real hardware**: switching Foxglove's
  Image panel from `camera/image_raw` to `camera/image_raw/compressed`
  dropped the lag from ~12s to ~1s.
- [ ] **Frame rate still stuck at ~12.6 Hz, not the format table's 30 Hz** —
  attempted fix (2026-07-13) via `CameraNode::request_frame_rate()`: a
  second, independent `open()` of the device node issuing `VIDIOC_S_PARM`
  before `camera_->start()`. **Confirmed on real hardware this does not
  work** — the ioctl reports success but the streamed rate is unchanged.
  The assumption that frame interval is a UVC device-level property, not
  per-fd state, was wrong for this driver: a second fd's successful
  `S_PARM` doesn't carry over to the fd that actually calls `STREAMON`.
  Real fix needs either the actual fd `V4l2CameraDevice` opens internally
  (not exposed — would need a fork/patch of `v4l2_camera`, a bigger step
  than attempted here) or a different mechanism entirely. Left in place
  since it's a harmless no-op on failure (`RCLCPP_WARN`), not because it's
  believed to work.
- [ ] **No calibration** — `camera_info_` is an uncalibrated placeholder
  (width/height/frame_id only, K/D/R/P all zero). Run `camera_calibration`
  and load the result via `camera_info_manager` if downstream consumers
  (AI/vision) need real intrinsics.
- [ ] **No tests** — unlike `mserve_drivechain`'s `test_packet_codec`, nothing
  here is unit-tested yet (there isn't much pure logic to test — most of the
  package is V4L2 device I/O — but the parameter validation in
  `camera_params.cpp` could be).
- [ ] **Mic (audio_common) blocked at the OS level, not attempted here** —
  `ros-lyrical-audio-capture` depends on `libgstreamer-plugins-good1.0-0`,
  which doesn't exist anywhere in this Ubuntu Resolute release's repos.
  Deferred; see `docs/session.md` for the investigation. If revisited,
  hand-rolling an ALSA capture node (matching this project's philosophy,
  and this package's own precedent of wrapping a device class directly
  instead of depending on someone else's node) publishing
  `audio_common_msgs/AudioData` is the likely path, since that message
  package installs fine without GStreamer.
- [ ] **Dockerfile** — not updated to install `v4l2_camera`/`web_video_server`;
  Docker path is legacy fallback only on this Pi (see top-level readme).

# mserve_camera — TODO

## Stage 1 — COMPLETE (2026-07-12)

- [x] Scaffold lifecycle node wrapping `v4l2_camera::V4l2CameraDevice`
- [x] Publish `sensor_msgs/Image` (`camera/image_raw`) + `sensor_msgs/CameraInfo` (`camera/camera_info`)
- [x] Wire into `lifecycle_manager`'s bringup/shutdown trees
- [x] Wire into `mserve_min.launch.py`
- [x] Add to `run_drivechain_hw.sh`'s process-cleanup lists (drivechain/base/camera/lifecycle_manager)
- [x] Web UI: `web/camera.html` — lifecycle, parameters, live image (via `web_video_server`), camera_info, logs
- [x] Web UI: image preview added to `web/base.html` too (for driving while watching the feed)
- [x] Verified end-to-end in `--sim` and with the real USB webcam: bringup → active → frames flowing → graceful shutdown

## Known limitations / next up

- [ ] **Frame rate stuck at ~12.6 Hz, not the format table's 30 Hz** — only pixel
  format + resolution are set via `requestDataFormat()`; the V4L2 frame
  *interval* (`VIDIOC_S_PARM`) is never explicitly requested, so the driver
  keeps whatever interval was last active. `V4l2CameraDevice` doesn't expose
  a wrapper for this ioctl — would need to open the fd directly or extend
  the wrapper. See `README.md`'s "Pixel format choice" section.
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

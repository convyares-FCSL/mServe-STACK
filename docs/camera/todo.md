# mserve_camera — known limitations

Package design and parameters: `ws/src/mserve_camera/README.md`.

- [ ] **Frame rate stuck at ~12.6 Hz, not the format table's 30 Hz.**
  `CameraNode::request_frame_rate()` (a second, independent `open()` issuing
  `VIDIOC_S_PARM` before `camera_->start()`) is verified on real hardware to
  be a silent no-op — the ioctl reports success but the streamed rate is
  unchanged; frame interval is per-fd state for this driver, and the fd that
  calls `STREAMON` is `V4l2CameraDevice`'s own, which isn't exposed. A real
  fix needs either a fork/patch of `v4l2_camera` or a different mechanism.
  The attempt is left in place because it's a harmless WARN-on-failure, not
  because it's believed to work.
- [ ] **No calibration** — `camera_info_` is an uncalibrated placeholder
  (width/height/frame_id only, K/D/R/P all zero). Run `camera_calibration`
  and load the result via `camera_info_manager` if downstream consumers
  (AI/vision) need real intrinsics.
- [ ] **No tests** — most of the package is V4L2 device I/O, but the
  parameter validation in `camera_params.cpp` is unit-testable.
- [ ] **Mic not implemented.** The camera has a capture-only mic. The
  original blocker (`audio-capture`'s GStreamer dependency missing from the
  native OS repos) belonged to the since-reverted native install —
  re-evaluate under Docker + Jazzy. Fallback: hand-rolled ALSA capture node
  publishing `audio_common_msgs/AudioData` (that message package has no
  GStreamer dependency), matching this package's wrap-the-device precedent.

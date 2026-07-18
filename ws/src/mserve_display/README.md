# mserve_display

Drives the ELEGOO 3.5" SPI touchscreen (ILI9486 display + ADS7846 resistive
touch, `dtoverlay=tft35a,rotate=90` — see repo root `transfer.md` for how
that got working on the Pi 5's RP1 chip). Talks to `/dev/fb0` and
`/dev/input/eventN` directly via plain Linux syscalls (`mmap`/`ioctl`/evdev
`read`) — no graphics library, no new apt dependency.

## Why not a lifecycle node?

Unlike `mserve_camera`/`mserve_lidar`/`mserve_drivechain`, this is a plain
`rclcpp::Node`. There's no hardware "connect" step worth gating — the
framebuffer and touch device are just opened once at construction, same
reasoning as `robot_state_publisher` (see `ws/src/launch/launch/
mserve_min.launch.py`'s comment on that node). It's not in
`lifecycle_manager`'s bringup/shutdown trees for the same reason.

## Screens

- **Face** (default on boot) — eyes track `/mserve/cmd_vel_safe`'s
  `angular.z` (positive = left). Tap anywhere → Menu.
- **Menu** — three buttons:
  - **Connect** — calls `/mserve_drivechain/connect` (`std_srvs/Trigger`,
    async). Shows the result, stays on Menu.
  - **Info** — IP address, drivechain/base lifecycle state, drive status,
    per-motor feedback. Tap anywhere → back to Menu.
  - **Face** — back to the Face screen.

## Architecture

```
DisplayNode (plain rclcpp::Node)
│
├── Framebuffer   /dev/fb0 access — mmap (falls back to write() if the
│                 driver doesn't support mmap), back-buffer + present()
├── TouchInput    finds "ADS7846 Touchscreen" in /proc/bus/input/devices
│                 at runtime (never hardcodes the event number — it shifts),
│                 non-blocking read polled by a timer, tap detection
├── screens.cpp   pure render functions (Face/Menu/Info) + button hit-testing,
│                 no ROS dependency
│
├── Subscriptions  /mserve/cmd_vel_safe, /mserve_drivechain/motor_feedback,
│                  /mserve_drivechain/drive_status, /mserve_base/base_status
├── Clients        /mserve_drivechain/connect (Trigger),
│                  /mserve_drivechain/get_state, /mserve_base/get_state
├── Service        ~/set_display_mode (interfaces/srv/SetDisplayMode) —
│                  lets other things force a screen change externally
└── Publisher      ~/status (interfaces/msg/DisplayStatus)
```

## Known-unverified (see `docs/TODO.md` / follow-up)

- **Touch calibration** (`touch_calib.*` params) ships as identity
  passthrough (raw ABS_X/Y 0-4095 mapped directly, no invert/swap) —
  not yet tuned against the actual rotated screen. Needs a live
  tap-the-corners session; update the params, no code change needed.
- **`mmap()` support** on this SPI fbdev driver hasn't been confirmed —
  `Framebuffer` probes for it and falls back to bulk `write()` if it fails.
- **Tap-vs-drag/tap-vs-noise thresholds** (`touch.tap_max_move`,
  `tap_min_hold_ms`, `tap_max_hold_ms`) are best-guess defaults.

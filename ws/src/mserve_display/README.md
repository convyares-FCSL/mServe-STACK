# mserve_display

Drives the ELEGOO 3.5" SPI touchscreen (ILI9486 display + ADS7846 resistive
touch, `dtoverlay=tft35a,rotate=90`). Talks to `/dev/fb0` and
`/dev/input/eventN` directly via plain Linux syscalls (`mmap`/`ioctl`/evdev
`read`) — no graphics library, no new apt dependency. Getting the overlay
itself working on the Pi 5's RP1 chip needed ELEGOO's actual precompiled
`tft35a.dtbo` (the generic mainline `fbtft` overlay loads but can't carry
the panel's manufacturer-specific init sequence); this panel's physical
mount also needed a full 180-degree software rotation on top of that overlay
(`fb_flip_180` param — see Framebuffer's constructor comment) to render
right-side up.

## Why not a lifecycle node?

Unlike `mserve_camera`/`mserve_lidar`/`mserve_drivechain`, this is a plain
`rclcpp::Node`. There's no hardware "connect" step worth gating — the
framebuffer and touch device are just opened once at construction, same
reasoning as `robot_state_publisher` (see `ws/src/launch/launch/
mserve_min.launch.py`'s comment on that node). It's not in
`lifecycle_manager`'s bringup/shutdown trees for the same reason.

## Screens

- **Face** (default on boot) — eyes (with eyebrows, so direction reads at a
  glance) track `/mserve/cmd_vel_safe`'s `angular.z` (positive = left, per
  REP-103). Tap anywhere → Menu.
- **Menu** — four buttons:
  - **Connect/Disconnect** — calls `/mserve_drivechain/connect`
    (`std_srvs/Trigger`, async). The label is a live status indicator (shows
    DISCONNECT once `drivechain_status.status` starts with `connected`), not
    a real toggle — `mserve_drivechain` has no disconnect capability yet, so
    tapping always calls connect regardless of which label shows (harmless
    no-op re-connect if already connected). Shows the result, stays on Menu.
  - **Info** — IP address (the host's real LAN IP even in Docker mode, see
    `getIpAddress()`'s `MSERVE_HOST_IP` env var check), WEB/GRAFANA links,
    drivechain/base lifecycle state, drive status, per-motor feedback. Tap
    anywhere → back to Menu.
  - **Face** — back to the Face screen.
  - **Calibrate** — a 4-tap Up/Down/Left/Right wizard that derives
    `touch_calib.*` (x/y range, invert, swap_xy) from the captured raw
    coordinates — see "Known-unverified" below, this is how the current
    values in `mserve_params.yaml` were produced. Tap-in order is debounced
    (`kCalibTapDebounceMs`) so an accidental double-tap on one prompt can't
    silently consume two steps.

Menu and Info auto-return to Face after `kMenuInfoTimeoutMs` (10s) of no
taps — Calibrate is exempt, since it's an active multi-step flow a timeout
would just interrupt.

## Architecture

```
DisplayNode (plain rclcpp::Node)
│
├── Framebuffer   /dev/fb0 access — mmap (falls back to write() if the
│                 driver doesn't support mmap), back-buffer + present()
├── TouchInput    finds "ADS7846 Touchscreen" in /proc/bus/input/devices
│                 at runtime (never hardcodes the event number — it shifts),
│                 non-blocking read polled by a timer, tap detection
├── screens.cpp   pure render functions (Face/Menu/Info/Calibrate) + button
│                 hit-testing, no ROS dependency
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

- **Touch calibration is done** — real values captured via the Calibrate
  wizard on this hardware are in `mserve_params.yaml`'s `touch_calib.*`
  (`swap_xy: true`, `invert_y: true` — this panel's raw axes are rotated
  relative to screen X/Y). Re-run the wizard (Menu → CALIBRATE) if touch
  hardware is ever reseated/replaced.
- **`mmap()` support** on this SPI fbdev driver — confirmed working on this
  hardware (`Framebuffer::open()` logs `mmap=yes`); the bulk-`write()`
  fallback path exists but isn't exercised on this unit.
- **Tap-vs-drag/tap-vs-noise thresholds** (`touch.tap_max_move`,
  `tap_min_hold_ms`, `tap_max_hold_ms`) are still best-guess defaults, not
  explicitly tuned.
- **Battery-tied eyes** — not yet implemented, see `docs/TODO.md`'s "Next"
  section: droop/close the Face screen's eyes as
  `drivechain_status.battery_level` falls.

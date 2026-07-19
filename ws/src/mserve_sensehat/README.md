# mserve_sensehat

Pi Sense HAT (2): 8x8 LED matrix, 5-way joystick, and LSM9DS1 IMU +
pressure/humidity sensors. Plain `rclcpp::Node`, not lifecycle-managed (same
reasoning as `mserve_display`/`mserve_joystick` — the kernel already owns
the framebuffer/input-event/I2C devices, this node just reads/writes them).

Debug UI: `web/sensehat.html`.

## What it does

- **IMU** — publishes `sensor_msgs/Imu` on `~/imu` (default
  `/mserve_sensehat/imu`) via [RTIMULib](https://github.com/RPi-Distro/RTIMULib)
  (vendored source, see `ws/src/third_party/README.md`). Orientation comes
  from RTIMULib's own sensor-fusion filter (accel+gyro+mag); standalone for
  now, not fused into `mserve_base`'s odometry.
- **Environmental sensors** — pressure (LPS25H) and humidity (HTS221),
  auto-discovered via the same `RTIMUSettings` mechanism as the IMU chip.
  Not published as their own topic — folded into `~/status` below.
- **`~/status`** (`interfaces/msg/SensehatStatus`) — one combined message for
  `web/sensehat.html`: temperature/pressure/humidity, accel/gyro/mag in
  human-friendly units (g, deg/s, µT — `~/imu` stays SI-only), compass
  heading, live joystick button state, `online`, and per-subsystem
  availability/calibration flags. Rate: `status_publish_hz` param.
- **LED matrix** — shows drivechain connection status: red X = disconnected,
  green O = connected. Subscribes to the same `drive_status` topic (and the
  same `status.rfind("connected", 0) == 0` check) as
  `mserve_display::screens.cpp`'s Menu screen.
- **Joystick** — `button_actions` param maps key names (`up`/`down`/`left`/
  `right`/`center`) to action names, same reassignable-mapping pattern as
  `mserve_joystick`'s `button_actions`. Only `connect` exists as an action
  right now (default: `center: connect`, calls `mserve_drivechain`'s
  `connect` service). All five keys' current pressed state is tracked and
  published on `~/status` regardless of whether they're mapped to anything.
- **`~/set_online`** (`std_srvs/SetBool`) — same `online`/`offline` pattern
  as `mserve_joystick`. Offline disables `button_actions` dispatch (so the
  joystick can be exercised without triggering `connect` etc.) — sensors,
  LED, and `~/status` keep working either way.

## Hardware quirk this package works around

Adding the Sense HAT changes **framebuffer numbering**: its `rpisense_fb`
driver ("RPi-Sense FB") can register ahead of the ELEGOO touchscreen's
`fb_ili9486` driver, bumping the touchscreen from `/dev/fb0` to `/dev/fb1`.
Both this package's `LedMatrix::resolveDevice()` and
`mserve_display`'s `resolveFramebufferDevice()` look their device up by
**driver name** via `/sys/class/graphics/fb*/name` instead of assuming a
fixed path — see `mserve_display/include/mserve_display/framebuffer.hpp`'s
comment for the exact incident that motivated this. `docker-compose.yml`
exposes both `/dev/fb0` and `/dev/fb1` (plus `/dev/i2c-1`) to the container
so this works regardless of which number lands where.

`LedMatrix::present()` also needs an explicit `lseek(fd_, 0, SEEK_SET)`
before every `write()` — this 8x8 device is only 128 bytes total, and
without seeking back, the *second* write lands past the end of the device
and silently fails (ignored `write()` return value hid it). First-ever draw
always "worked" regardless, since a freshly-opened fd starts at offset 0 —
confirmed on real hardware: the icon simply never changed after boot until
this was added.

## Calibration

`~/status`'s `compass_calibrated`/`accel_calibrated` fields (and
`web/sensehat.html`'s Device Availability panel) report whether RTIMULib
actually has calibration data loaded (`RTIMU::getCompassCalibrationValid()`/
`getAccelCalibrationValid()`) — **not** just whether the chip is present.
Both are `false` until calibration is run once; `heading_deg` is unreliable
while `compass_calibrated` is `false` (confirmed on real hardware: ~174°
reported while pointed in a direction a phone compass read very
differently — this chassis has DDSM115 hub motors, which contain strong
magnets, likely near the HAT).

RTIMULib ships `RTIMULibCal` (`ws/src/third_party/RTIMULib/Linux/RTIMULibCal/`)
for exactly this — an interactive terminal tool, not something this node
runs itself. It writes to `<cwd>/RTIMULib.ini` using the 1-arg
`RTIMUSettings` constructor, so it must be run **with `sensehat_calibration/`
(repo root) as the working directory** — that directory is bind-mounted
into the container at `imu_settings_dir` (default `/tmp/mserve_sensehat`,
see `docker-compose.yml`), so the node picks up whatever's written there.

**Run this on the host, not inside the container** — `RTIMULibCal` is an
interactive terminal tool (raw keypresses, live on-screen feedback while you
physically move the robot), which doesn't work well through
`docker compose exec`. The host already has `librtimulib-dev` installed
(Raspberry Pi OS package); compile straight against it:

```bash
g++ -o /tmp/RTIMULibCal ws/src/third_party/RTIMULib/Linux/RTIMULibCal/RTIMULibCal.cpp -lRTIMULib

cd sensehat_calibration   # repo root — bind-mounted into the container, NOT
                          # the container's own /tmp (a container's /tmp is
                          # not the host's — real calibration data was lost
                          # this way once already, run against a plain host
                          # /tmp dir with no bind mount)
/tmp/RTIMULibCal
```

The ellipsoid fit step (`e`) shells out to `octave`
(`sudo apt install octave` if missing) and needs `RTEllipsoidFit.m`/
`ellipsoid_fit.m`/`mag_cal.m`/`mag_fit_display.m` — copy them from
`/usr/share/librtimulib-utils/RTEllipsoidFit/` (same apt package) into
`sensehat_calibration/` first; RTIMULibCal hardcodes that absolute path for
locating the scripts themselves, but Octave's internal `source()` calls
resolve bare filenames against the current directory, not that path, so it
fails to find them unless a copy sits right next to your `RTIMULib.ini`.

Magnetometer min/max calibration: waggle/rotate the robot so all six axes
(+x/-x/+y/-y/+z/-z) hit their extrema, then `s` to save. On a wheeled ground
robot this is awkward — Z extrema in particular needs physically tilting
the whole chassis, not just spinning it in place — do your best, partial
coverage still beats no calibration. Ellipsoid calibration (finer, needs
many poses across 8 octants) is optional; min/max alone is the main fix.
Restart `mserve_sensehat` afterward to pick up the new `RTIMULib.ini`.

If the heading still looks systematically rotated after calibration (not
noisy, just offset by a consistent amount), the next thing to check is
`RTIMUSettings`' `m_axisRotation` (default `RTIMU_XNORTH_YEAST` — assumes
the chip's X-axis faces the robot's front) against how the HAT is actually
seated on the GPIO header.

## Params (`mserve_params.yaml`)

| Param | Default | Meaning |
|---|---|---|
| `imu_frame_id` | `sensehat_link` | TF frame on published `Imu` messages — not yet a real URDF frame |
| `imu_publish_hz` | 20.0 | IMU poll/publish rate |
| `imu_settings_dir` | `/tmp/mserve_sensehat` | RTIMULib's own settings/calibration cache (chip type + I2C address it auto-discovered, plus calibration data — see Calibration above) |
| `joystick_poll_hz` | 20.0 | Joystick evdev poll rate |
| `joystick_device_name_match` | `Raspberry Pi Sense HAT Joystick` | evdev device name to find via `/proc/bus/input/devices` |
| `status_publish_hz` | 5.0 | `~/status` publish rate |
| `button_actions.<key>` | `center: connect` | key (up/down/left/right/center) -> action name; only `connect` exists so far |
| `topic_names.imu` | `/mserve_sensehat/imu` | |
| `topic_names.status` | `/mserve_sensehat/status` | |
| `topic_names.drivechain_status` | `/mserve_drivechain/drive_status` | |
| `service_names.connect` | `/mserve_drivechain/connect` | |

## Not yet done

- IMU isn't fused into `mserve_base`'s odometry (see `docs/TODO.md`) — this
  package only publishes raw `sensor_msgs/Imu`.
- Color/brightness sensor (TCS3400) isn't read.
- Up/down/left/right joystick directions have no `button_actions` mapped by
  default (only `center` does) — the mechanism supports it, nothing else
  uses it yet.

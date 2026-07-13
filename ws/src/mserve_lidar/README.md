# mserve_lidar

Lifecycle-managed LiDAR node for mServe. Wraps the vendored Slamtec RPLIDAR
SDK's `sl::ILidarDriver` directly (raw serial device access, not the whole
`rplidar_ros` node) so bringup/shutdown is gated the same way as
`mserve_drivechain`/`mserve_base`/`mserve_camera`, via `lifecycle_manager`.
Targets the RPLIDAR C1.

## Why not just depend on `rplidar_ros`?

Two reasons, one of them stronger than the equivalent case for `mserve_camera`:

1. **Same reasoning as camera**: `rplidar_ros`'s node (`rplidar_node.cpp`) is a
   plain `rclcpp::Node`, not a lifecycle node, so it can't be
   configured/activated in step with the rest of the stack or included in
   `lifecycle_manager`'s bringup/shutdown trees.
2. **Unlike camera, there's no apt package to link against at all.** This
   distro's ROS apt repo has no `ros-lyrical-rplidar-ros` (verified against
   `packages.ros.org`) — and even upstream's own `CMakeLists.txt` doesn't
   build the SDK as a reusable library; it just globs `sdk/src/*.cpp` straight
   into its node executable. There's nothing to `find_package()`.

So `sdk/` — Slamtec's RPLIDAR SDK, a clean, ROS-free C++ library with its own
`sl::ILidarDriver` interface (`connect()`/`startScan()`/`grabScanDataHq()`/
`stop()`) — is vendored into `third_party/rplidar_sdk/` (from the `ros2`
branch of [slamtec/rplidar_ros](https://github.com/slamtec/rplidar_ros), BSD
2-clause, see `third_party/rplidar_sdk/LICENSE`) and `mserve_lidar` wraps that
directly, the same way `mserve_camera` wraps `V4l2CameraDevice` and
`mserve_drivechain` wraps `DriveUart`.

TCP/UDP channel support and the legacy pre-`sl::` A/A2/A3 driver
(`rplidar_driver.cpp`, superseded by `sl_lidar_driver.cpp`) were dropped from
the vendored copy — the C1 only ever needs the modern `sl::` API over a serial
channel, and there's no reason to build code this project never calls.

## Architecture

```
LidarNode (lifecycle)
│
├── on_configure   sl::createLidarDriver() + createSerialPortChannel(),
│                  connect(), getDeviceInfo()/getHealth() sanity checks
├── on_activate    setMotorSpeed() + startScan()/startScanExpress(),
│                  starts a dedicated capture thread
├── on_deactivate  stops the capture thread, stop() + setMotorSpeed(0)
├── on_cleanup     disconnect() + delete driver/channel
├── on_shutdown    same as cleanup — safe to call from any state
│
└── Publisher  scan   (sensor_msgs/LaserScan)
```

Like `mserve_camera`'s `capture_loop()` around a blocking `capture()` call,
there's no BehaviorTree here — `grabScanDataHq()` blocks until a full
revolution is ready, so a dedicated capture thread is enough; there's no
multi-step hardware handshake needing retries/sequencing once the device is
connected.

### Key source files

| File | Purpose |
|---|---|
| `include/mserve_lidar/lidar_node.hpp` | Node public API |
| `include/mserve_lidar/lidar_limits.hpp` | Named constants (baudrate, min range) |
| `src/lidar_node.cpp` | Lifecycle callbacks, SDK connection helpers, capture loop |
| `src/lidar_params.cpp` | Parameter declaration, loading, hot-change guard |
| `src/main.cpp` | Entry point |
| `third_party/rplidar_sdk/` | Vendored Slamtec RPLIDAR SDK (BSD-licensed) |
| `docs/lidar/todo.md` (repo root) | Known limitations, next steps |

## Parameters

| Parameter | Default | Notes |
|---|---|---|
| `device` | `/dev/ttyUSB0` | Serial port |
| `baudrate` | `460800` | Fixed for the C1 — not configurable on the device side |
| `frame_id` | `lidar_link` | Matches `mserve_lidar.xacro`'s sensor link |
| `scan_mode` | `""` (empty) | Empty = device's typical/default scan mode; set to a mode name from `getAllSupportedScanModes()` to force a specific one |
| `inverted` | `false` | Reverses scan direction — set `true` if the unit is mounted upside-down |

`device`/`baudrate` can only be changed in `UNCONFIGURED` state (reconnecting
mid-stream isn't supported).

## Scan message construction

`ranges[]`/`angle_min`/`angle_max` follow the same simplification every
RPLIDAR ROS driver uses: after `ascendScanData()` sorts one revolution's
points into ascending angle order, leading/trailing no-return samples are
trimmed, and the remaining points are treated as evenly spaced in angle from
the first sample's measured angle to the last's — even though the SDK's
actual per-sample angle spacing isn't perfectly uniform. This is what
`sensor_msgs/LaserScan` requires (`angle_increment` is a single scalar) and
what Nav2/SLAM tooling expects as input; the error this introduces is well
below the sensor's own noise floor at the C1's ~500-1000 points/revolution.

Angle-compensation (redistributing samples into fixed 1-degree bins,
duplicating/dropping points to fill gaps — what `rplidar_ros` calls
`angle_compensate`) is intentionally **not** implemented — it's a denser but
lossier representation of the same data, and skipping it keeps the capture
loop a single straight-line pass matching this project's "every part
visible" philosophy. Revisit if a downstream consumer actually needs
fixed-angle bins.

`range_max` is clamped to `kMaxRangeM` (12.0m, the C1's datasheet spec —
`lidar_limits.hpp`), not published straight from the SDK's negotiated scan
mode. `getAllSupportedScanModes()`/`startScan()` report a `max_distance` per
scan mode (e.g. `DenseBoost` reports 40.0m) that describes the sample-rate
tradeoff for that mode, not a promise of usable returns that far — publishing
it unclamped would make RViz/the web UI scale their views around a range the
sensor can't actually deliver.

## Build

```bash
cd ~/mServe-STACK/ws
colcon build --packages-select interfaces utils mserve_lidar --cmake-args -DBUILD_TESTING=OFF --symlink-install
source install/setup.bash
```

No extra `find_package()`s needed beyond `rclcpp`/`rclcpp_lifecycle`/
`sensor_msgs`/`utils` — the vendored SDK builds as a plain static library
(`rplidar_sdk`, see `CMakeLists.txt`) from `third_party/rplidar_sdk/src/`,
needing only `Threads::Threads`.

## Device permissions

`/dev/ttyUSB0` is normally owned by group `dialout`. If the node fails to
connect with a permissions error:

```bash
sudo usermod -aG dialout $USER
newgrp dialout   # or log out/in — group membership doesn't apply to already-open shells
```

## Run

### Via launch (recommended)

```bash
cd ~/mServe-STACK/ws
source install/setup.bash
ros2 launch launch mserve_min.launch.py backend:=sim
```

`lidar_node` is launched alongside `mserve_drivechain`/`mserve_base`/
`mserve_camera`, and `lifecycle_manager` configures/activates it as part of
the same bringup tree. `scripts/run_stack.sh` is the normal entry point — see
the top-level readme.

### Manually

```bash
source install/setup.bash
ros2 run mserve_lidar lidar_node

# separate terminal
ros2 lifecycle set /mserve_lidar configure
ros2 lifecycle set /mserve_lidar activate

ros2 topic hz /scan
ros2 topic echo /scan --field ranges
```

## Web UI

`web/lidar.html` shows lifecycle state, parameters, a live top-down scan plot
(2D canvas, no external deps — unlike the camera's MJPEG stream, `LaserScan`
is plain JSON over rosbridge, so no `web_video_server`-style transcode is
needed), scan stats (point count, measured rate, range), and `/rosout` logs,
matching the pattern of `web/camera.html`. See the top-level `web/README.md`.

## Tests

None yet — see `docs/lidar/todo.md`.

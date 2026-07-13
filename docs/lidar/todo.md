# mserve_lidar — TODO

## Stage 1 — COMPLETE (2026-07-13)

- [x] Vendor Slamtec RPLIDAR SDK (`third_party/rplidar_sdk`, from
  `slamtec/rplidar_ros`'s `ros2` branch) — no apt package exists for this
  distro, and upstream's own build has no reusable library target, so the
  `sdk/` sources are copied in directly (serial-only, modern `sl::` API only)
- [x] Scaffold lifecycle node wrapping `sl::ILidarDriver`
- [x] Publish `sensor_msgs/LaserScan` (`scan`)
- [x] Wire into `lifecycle_manager`'s bringup/shutdown trees
- [x] Wire into `mserve_min.launch.py` + `interfaces/config/mserve_params.yaml`
- [x] Add to `run_stack.sh`'s process-cleanup lists (drivechain/base/camera/lidar/lifecycle_manager)
- [x] Web UI: `web/lidar.html` — lifecycle, parameters, live 2D scan plot (canvas), scan stats, logs
- [x] URDF/Gazebo (`mserve_lidar.xacro`, `mserve_gazebo_bridge.yaml`) — was already
  scaffolded before this node existed; node's `frame_id`/topic defaults
  (`lidar_link`/`scan`) and range (`0.05`-`12.0`m) were matched to it, not
  the other way around
- [x] Verified end-to-end against a physical RPLIDAR C1 (`/dev/ttyUSB0`, CP210x
  USB-serial adapter): `configure` reads real device info (firmware 1.02,
  hardware rev 18), `activate` spins the motor via the `DEFAULT_MOTOR_SPEED`
  sentinel and negotiates `DenseBoost` scan mode with no explicit RPM/PWM
  needed, `/scan` streams at ~10.3 Hz with coherent ranges. Cycled
  `deactivate` → `cleanup` and, separately, `shutdown` called directly from
  `active` (the exact transition `lifecycle_manager`'s shutdown tree uses) —
  both clean, no crashes, motor stops each time.

## Known limitations / next up

- [ ] **No angle compensation** — see README.md's "Scan message construction"
  section. `ranges[]` is built directly from one revolution's raw ascending
  samples (evenly-spaced-angle approximation, same one every RPLIDAR ROS
  driver uses), not redistributed into fixed 1-degree bins. Revisit if a
  downstream consumer (Nav2 costmap, scan matcher) turns out to need that.
- [ ] **No tests** — unlike `mserve_drivechain`'s `test_packet_codec`, nothing
  here is unit-tested yet (most of the package is SDK/serial I/O, but the
  parameter validation in `lidar_params.cpp` could be).
- [ ] **Dockerfile** — not updated to install anything lidar-related; Docker
  path is legacy fallback only on this Pi (see top-level readme), and
  `run_stack.sh`'s Docker build branch was never updated for
  `mserve_camera` either — same gap, not new to this package.

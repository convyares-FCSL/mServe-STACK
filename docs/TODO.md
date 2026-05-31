# TODO

## Now
- [x] Scaffold `scripts/` hygiene and build helpers.
- [x] Scaffold `mserve_interfaces` with central config and first messages.
- [x] Scaffold `mserve_utils` with config/QoS validation tests.
- [x] Scaffold `mserve_base` lifecycle node.
- [x] Scaffold `mserve_drivetrain` lifecycle node in dry-run mode.
- [x] Scaffold `mserve_bringup` minimal launch.
- [ ] Decide first ESP32 transport: USB serial, UART, UDP, or other.
- [ ] Decide whether first robot model is plain URDF or minimal Xacro.

## Next

- [ ] Add launch tests for minimal bringup.
- [ ] Add robot description and TF smoke test.
- [ ] Add Gazebo simulation package.
- [ ] Add lidar package.
- [ ] Add camera package.
- [ ] Add display package.
- [ ] Add Nav2 simulation launch.

## Later

- [ ] Add real ESP32 packet protocol.
- [ ] Add hardware safety behavior for ESP32 comms loss.
- [ ] Add AI package.
- [ ] Add arm/manipulation package.
- [ ] Revisit `ros2_control` later as a comparison exercise, not as the first implementation.

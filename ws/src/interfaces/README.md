# interfaces

Shared ROS 2 messages, services, actions, and central YAML config for the
mServe stack. Package name is `interfaces` (dropped the originally-planned
`mserve_interfaces` prefix, like `utils` and `launch` — see root `readme.md`).

Everything below is what's actually defined here today. (An earlier version
of this README documented a HyFleet compressor/booster interface set —
`ControlCompressor`/`ControlBooster` actions, `BoosterCmd`/`CompressorCmd`/
`SetMode`/`DispenserCmd`/`GasRouterCmd` services, `CompressorTelemetry` — none
of that exists anymore; it was removed as unrelated HyFleet-derived
scaffolding, see `docs/TODO.md`'s "Now" section.)

## Messages (`msg/`)

- **`DriveStatus.msg`** — `string status`, `float32 battery_level`, `bool
  board_alive`. Published by both `mserve_drivechain` and `mserve_base`
  (`status` values like `connected_hw`/`connected_sim`/`idle_hw`/`idle_sim`
  for drivechain — see `mserve_drivechain/src/drivechain_bt_nodes.cpp`).
- **`MotorState.msg`** — per-motor feedback: `motor_id`, `name`,
  `velocity_rpm`, `velocity_rads`, `position_rad`, `current_a`,
  `temperature_c`, `fault_code`.
- **`DriveMotorFeedback.msg`** — `builtin_interfaces/Time stamp` + an array
  of `MotorState` (one per motor). Published by `mserve_drivechain`.
- **`MotorCommand.msg`** — `motor_id`, `int16 rpm`. Used inside
  `srv/Drive.srv`'s request array, not published standalone.
- **`DisplayStatus.msg`** — `string mode`, `string text`. Published by
  `mserve_display` on `~/status` (mode = current screen name, text = a
  short status string).
- **`Esp32Status.msg`** — `bool connected`, `float32 temperature`, `string
  firmware_version`. Reserved for future ESP32 health reporting; not yet
  published anywhere.

## Services (`srv/`)

- **`Drive.srv`** — request: `MotorCommand[] motor_commands`; response:
  `bool success`, `string message`. `mserve_drivechain`'s low-level motor
  command interface.
- **`SetDisplayMode.srv`** — request: `string mode`; response: `bool
  success`, `string message`. `mserve_display`'s `~/set_display_mode` —
  forces a screen change (`face`/`menu`/`info`/`calibrate`) independent of
  touch navigation.
- **`SetMotorId.srv`** — request: `uint8 motor_id`, `uint8 new_id`;
  response: `bool success`, `string message`. Reassigns a DDSM115 motor's
  hardware ID.

`mserve_drivechain`'s `~/connect` and `mserve_base`/`mserve_drivechain`'s
`~/get_state` use the standard `std_srvs/srv/Trigger` and
`lifecycle_msgs/srv/GetState` — not custom interfaces, nothing to define
here.

## Actions (`action/`)

- **`Dock.action`** — goal: `string dock_point`; result: `bool success`,
  `string message`; feedback: `float32 progress`. Reserved for future
  docking behavior; not yet implemented by any node.

## Central config (`config/`)

- **`mserve_params.yaml`** — every node's runtime parameters in one file,
  nested by node name (`mserve_base`, `mserve_drivechain`, `mserve_camera`,
  `mserve_lidar`, `mserve_display`), dot-notation nested keys. Loaded via
  `--params-file` in the launch files.
- **`qos.yaml`** / **`topics.yaml`** — named QoS profiles and topic names,
  read by `utils`' `mserve_qos`/`mserve_topics` helpers rather than hardcoded
  per-node.
- **`slam_params_map.yaml`** / **`slam_params_local.yaml`** — SLAM Toolbox
  params, split by mode (`mserve_slam.launch.py`'s `mode:=map`/`mode:=local`).

## Build

```bash
colcon build --packages-select interfaces
source install/setup.bash

# Inspect any interface
ros2 interface show interfaces/srv/SetDisplayMode
ros2 interface show interfaces/msg/DriveStatus
```

# mserve_drivechain

Controls two DDSM115 brushless motors via a Waveshare **DDSM Driver HAT (A)** mounted on a Raspberry Pi.

## Architecture

```
DrivechainNode (lifecycle)
│
├── Service  ~/mserve_drivechain/drivechain_cmd  ← CONNECT / STOP / SET_ID
│     └── BT trees: connect_tree, stop_tree, set_id_tree
│
├── Subscription  /cmd_vel  (geometry_msgs/Twist)
│     └── DiffDrive kinematics → left_rpm / right_rpm → blackboard
│
├── Timer  10 Hz drive loop
│     └── DriveUart::set_speed() → UART → motors → feedback → publish
│
├── Publisher  ~/mserve_drivechain/wheel_feedback  (WheelFeedback)
└── Publisher  ~/mserve_drivechain/drive_status    (DriveStatus)
```

The **blackboard** connects everything:

| Key | Written by | Read by |
|---|---|---|
| `uart` | on_configure | BT nodes |
| `uart_connected` | OpenUart / CloseUart | drive timer |
| `uart_device` | load_params | OpenUart BT node |
| `left_motor_id` / `right_motor_id` | load_params | BT nodes + drive timer |
| `left_rpm` / `right_rpm` | on_cmd_vel | drive timer |
| `left_fault` / `right_fault` | drive timer | MotorHealthy condition |
| `target_motor_id` / `new_motor_id` | service handler | set_id tree |

## Hardware: DDSM Driver HAT (A) on Raspberry Pi

The HAT sits on the 40-pin GPIO header and communicates over the Pi's **hardware UART** — no USB required.

### One-time Pi setup

```bash
sudo raspi-config
# → Interface Options → Serial Port
#   "Would you like a login shell over serial?" → No
#   "Would you like the serial port hardware enabled?" → Yes
sudo reboot
```

After reboot the UART is available at `/dev/serial0` (symlink to `/dev/ttyAMA0` on Pi 4).

### Motor IDs

Each DDSM115 on the RS-485 bus needs a unique ID. Factory default is ID=1 for both motors, so you must assign them before connecting. Connect **one motor at a time** and use the SET_ID command:

```bash
# Assign left motor ID=1 (factory default — just verifies)
ros2 service call /mserve_drivechain/drivechain_cmd mserve_interfaces/srv/DriveChainCmd \
  "{command: 3, motor_id: 1, new_id: 1}"

# Swap motor, assign right motor ID=2
ros2 service call /mserve_drivechain/drivechain_cmd mserve_interfaces/srv/DriveChainCmd \
  "{command: 3, motor_id: 1, new_id: 2}"
```

## Parameters

| Parameter | Default | Notes |
|---|---|---|
| `drive.backend` | `"sim"` | `"sim"` = Gazebo/no hardware; `"hardware"` = real HAT |
| `hardware.uart_device` | `"/dev/serial0"` | UART device path |
| `hardware.left_motor_id` | `1` | DDSM115 bus ID for left wheel |
| `hardware.right_motor_id` | `2` | DDSM115 bus ID for right wheel |
| `hardware.wheel_separation` | `0.35` | Centre-to-centre (m) |
| `hardware.wheel_radius` | `0.08` | Wheel radius (m) |
| `drive.max_rpm` | `200` | Max speed (DDSM115 ceiling is 200 RPM) |
| `drive.cmd_vel_timeout_ms` | `500` | Zero motors if no cmd_vel for this long |
| `feedback_rate` | `10.0` | Drive loop / publish rate (Hz) |

Hardware params (`drive.backend`, `hardware.*`, `drive.max_rpm`) can only be changed in `UNCONFIGURED` state.
`feedback_rate` and `drive.cmd_vel_timeout_ms` are hot-changeable at runtime.

## Build

```bash
colcon build --packages-select mserve_interfaces mserve_drivechain
source install/setup.bash
```

## Run

### Sim / Gazebo mode (default)

```bash
ros2 run mserve_drivechain drivechain_node
```

In another terminal, bring the node through its lifecycle:

```bash
ros2 lifecycle set /mserve_drivechain configure
ros2 lifecycle set /mserve_drivechain activate

# Connect (sim: always succeeds immediately)
ros2 service call /mserve_drivechain/drivechain_cmd mserve_interfaces/srv/DriveChainCmd \
  "{command: 1}"

# Drive
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Stop and close
ros2 service call /mserve_drivechain/drivechain_cmd mserve_interfaces/srv/DriveChainCmd \
  "{command: 2}"
```

### Real hardware

```bash
ros2 run mserve_drivechain drivechain_node \
  --ros-args -p drive.backend:=hardware -p hardware.uart_device:=/dev/serial0

ros2 lifecycle set /mserve_drivechain configure
ros2 lifecycle set /mserve_drivechain activate
ros2 service call /mserve_drivechain/drivechain_cmd mserve_interfaces/srv/DriveChainCmd \
  "{command: 1}"
```

### Monitor feedback

```bash
ros2 topic echo /mserve_drivechain/wheel_feedback
ros2 topic echo /mserve_drivechain/drive_status
```

## Service reference

Service: `~/drivechain_cmd` (`mserve_interfaces/srv/DriveChainCmd`)

| command | value | extra fields | effect |
|---|---|---|---|
| `CONNECT` | 1 | — | Open UART, ping motors, set speed mode |
| `STOP` | 2 | — | Zero motors, close UART |
| `SET_ID` | 3 | `motor_id`, `new_id` | Change motor hardware ID (one motor on bus only) |

## BT trees

| Tree | File | Triggered by |
|---|---|---|
| Connect | `connect_tree.xml` | CONNECT service call |
| Stop | `stop_tree.xml` | STOP service call or on_deactivate |
| Set ID | `set_id_tree.xml` | SET_ID service call |

## Tests

```bash
colcon test --packages-select mserve_drivechain --event-handlers console_direct+
```

- `test_diff_drive` — kinematics: straight, reverse, pivot, arc
- `test_packet_codec` — CRC-8/MAXIM verification + sim-mode DriveUart round-trips

## DDSM115 protocol notes

- 115200 baud, RS-485 half-duplex on single bus
- 10-byte packets: CRC-8/MAXIM (polynomial 0x8C) over bytes [0..8], stored in byte [9]
- Speed loop (mode=2): RPM range −200 to +200
- Feedback: mode, torque, speed_rpm, position (0–32767 = 0–360°), fault_code
- Mode change quirk: byte[9] = mode value directly, no CRC
- ID change: send 5× with 4ms gaps, then verify with ping

# mserve_drivechain

Controls two DDSM115 brushless motors via a Waveshare **DDSM Driver HAT (A)** mounted on a Raspberry Pi.

## Architecture

```
DrivechainNode (lifecycle)
│
├── Service  ~/drivechain_cmd  ← CONNECT / STOP / SET_ID
│     └── BT trees: connect_tree, stop_tree, set_id_tree (run synchronously under uart_mutex)
│
├── Subscription  /cmd_vel  (geometry_msgs/Twist)
│     └── CmdVelCache — always holds latest Twist + steady_clock timestamp
│
├── Timer  (feedback_rate Hz, default 10 Hz)
│     └── drive_tree.xml: UartOpen → ComputeRpm → SetMotorSpeed×2 → Publish×2
│           ComputeRpm: diff-drive kinematics + cmd_vel watchdog → left_rpm / right_rpm
│           SetMotorSpeed: UART → motor feedback → blackboard → publishers
│
├── Publisher  ~/wheel_feedback  (WheelFeedback)
└── Publisher  ~/drive_status    (DriveStatus)
```

The **blackboard** is the single source of truth shared between the node and all BT nodes:

| Key | Type | Written by | Read by |
|---|---|---|---|
| `uart` | `DriveUart*` | on_configure | BT nodes |
| `cmd_vel_cache` | `CmdVelCache*` | on_configure | ComputeRpm |
| `uart_connected` | bool | OpenUart / CloseUart | PublishDriveStatus |
| `uart_device` | string | load_params | OpenUart |
| `left_motor_id` / `right_motor_id` | int | load_params | BT nodes |
| `wheel_separation` / `wheel_radius` | double | load_params | ComputeRpm |
| `max_rpm` / `cmd_vel_timeout_ms` | int | load_params | ComputeRpm |
| `feedback_rate` | double | load_params | on_activate, on_parameters |
| `left_rpm` / `right_rpm` | int | ComputeRpm | SetMotorSpeed |
| `left_speed_fb` / `right_speed_fb` | double (rad/s) | SetMotorSpeed | PublishWheelFeedback |
| `left_pos_fb` / `right_pos_fb` | double (rad) | SetMotorSpeed | PublishWheelFeedback |
| `left_fault` / `right_fault` | int | SetMotorSpeed | MotorHealthy condition |
| `target_motor_id` / `new_motor_id` | int | service handler | set_id_tree |
| `publish_wheel_feedback` | `std::function` | on_configure | PublishWheelFeedback |
| `publish_drive_status` | `std::function` | on_configure | PublishDriveStatus |

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

Each DDSM115 on the RS-485 bus needs a unique ID. Factory default is ID=1 for both motors, so assign them before first use. Connect **one motor at a time** and use the SET_ID command:

```bash
# With only the left motor connected — confirm it's ID 1 (factory default)
ros2 service call /mserve_drivechain/drivechain_cmd mserve_interfaces/srv/DriveChainCmd "{command: 3, motor_id: 1, new_id: 1}"

# Swap to the right motor — reassign from ID 1 → ID 2
ros2 service call /mserve_drivechain/drivechain_cmd mserve_interfaces/srv/DriveChainCmd "{command: 3, motor_id: 1, new_id: 2}"
```

## Parameters

| Parameter | Default | Notes |
|---|---|---|
| `drive.backend` | `"sim"` | `"sim"` = no hardware; `"hardware"` = real HAT |
| `hardware.uart_device` | `"/dev/serial0"` | UART device path |
| `hardware.left_motor_id` | `1` | DDSM115 bus ID for left wheel |
| `hardware.right_motor_id` | `2` | DDSM115 bus ID for right wheel |
| `hardware.wheel_separation` | `0.35` | Centre-to-centre distance (m) |
| `hardware.wheel_radius` | `0.08` | Wheel radius (m) |
| `drive.max_rpm` | `200` | Max speed — DDSM115 ceiling is 200 RPM |
| `drive.cmd_vel_timeout_ms` | `500` | Zero motors if no cmd_vel received for this long |
| `feedback_rate` | `10.0` | Drive loop / publish rate (Hz) |

Hardware params (`drive.backend`, `hardware.*`, `drive.max_rpm`) can only be changed in `UNCONFIGURED` state.
`feedback_rate` and `drive.cmd_vel_timeout_ms` are hot-changeable at runtime.

## Build

```bash
colcon build --packages-select mserve_interfaces mserve_drivechain
source install/setup.bash
```

## Web UI

A browser-based control panel is in `web/` at the repo root. It connects to ROS via rosbridge and provides lifecycle controls, motor connect/stop, a D-pad with speed sliders, and live wheel feedback.

```
web/
├── drivechain.html     — drivechain control page
├── drivechain.js
├── index.html          — main debug bridge (links to drivechain page)
├── run_drivechain_hw.sh — one-command stack launcher (sim or hardware)
└── roslib.min.js
```

### Quick launch (recommended)

The `run_drivechain_hw.sh` script starts rosbridge, the drivechain node, runs the lifecycle, and serves the web UI — all in one command. Press **Ctrl+C** to cleanly stop everything.

**The script auto-detects whether ROS 2 is installed natively.** If `ros2` is not found (e.g., on a Raspberry Pi running Debian without ROS), it falls back automatically to Docker — starting the container, building the packages inside it, and routing all ROS commands through `docker compose exec`.

**Sim (no hardware needed):**
```bash
cd ~/ai-workspace/projects/mServe-STACK
./web/run_drivechain_hw.sh --sim
# Open: http://localhost:8080/drivechain.html
```

**Real hardware (Pi with HAT):**
```bash
cd ~/ai-workspace/projects/mServe-STACK
# Ensure Pi UART is configured (see One-time Pi setup) and motor IDs are set
./web/run_drivechain_hw.sh
# Open: http://<pi-ip>:8080/drivechain.html

# Custom UART device:
./web/run_drivechain_hw.sh /dev/ttyAMA0
```

The browser banner shows `[sim]` or `[hardware (/dev/serial0)]` so you can tell at a glance which mode is running.

### Running on a Raspberry Pi (Debian / no native ROS)

The Pi typically has no ROS installation. The Docker stack handles everything:

**Prerequisites (one-time):**
```bash
# Install Docker
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
# Log out and back in so the group takes effect

# Enable the hardware UART (if not already done — see One-time Pi setup above)
```

**Clone and run:**
```bash
git clone <repo-url> ~/mServe-STACK
cd ~/mServe-STACK

# First run: Docker builds the image and compiles the ROS packages (takes a few minutes)
./web/run_drivechain_hw.sh

# Subsequent runs: incremental build, much faster
```

On the Pi, `run_drivechain_hw.sh` automatically:
1. Detects that `ros2` is not available natively
2. Starts the `robot-mserve` Docker container
3. Runs `colcon build` inside the container
4. Launches rosbridge and the drivechain node inside the container
5. Runs the lifecycle transitions inside the container
6. Serves the web UI natively with Python on port 8080

The web server always runs natively (Python is always available on Debian), so the browser URL is simply `http://<pi-ip>:8080/drivechain.html` from any machine on the same network.

> **Serial device passthrough**: `docker-compose.yml` maps `/dev/serial0` into the container and adds the `dialout` group. If your Pi uses a different device (e.g., `/dev/ttyAMA0`), pass it as an argument: `./web/run_drivechain_hw.sh /dev/ttyAMA0` — but also update `docker-compose.yml` devices accordingly.

### Web UI controls

| Control | Action |
|---|---|
| Configure / Activate / Deactivate | Lifecycle transitions |
| **Connect motors** | Calls CONNECT service — opens UART, pings motors, sets speed mode |
| **Stop motors** | Calls STOP service — zeros motors, closes UART |
| Linear / Turn rate sliders | Sets max speed for D-pad buttons |
| ▲ ▼ ◄ ► | Hold to drive in that direction; release to stop |
| ■ | Immediate stop (zero cmd_vel) |
| W / A / S / D or arrow keys | Same as D-pad, hold to move |
| Space | Stop |

The drive status badge updates live: `idle_sim`, `connected_sim`, `idle_hw`, `connected_hw`.

---

## Run (manual / CLI)

> **Every new terminal must source the workspace before using `ros2 service call`.**
> The lifecycle commands work without sourcing but the service type lookup does not.
> ```bash
> source install/setup.bash
> ```

---

### Sim mode (default — no hardware needed)

**Terminal 1 — node:**
```bash
source install/setup.bash
ros2 run mserve_drivechain drivechain_node
```

**Terminal 2 — lifecycle + control:**
```bash
source install/setup.bash

ros2 lifecycle set /mserve_drivechain configure
ros2 lifecycle set /mserve_drivechain activate

# Connect (sim: always succeeds immediately)
ros2 service call /mserve_drivechain/drivechain_cmd mserve_interfaces/srv/DriveChainCmd "{command: 1}"

# Drive forward at 0.2 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Stop and close
ros2 service call /mserve_drivechain/drivechain_cmd mserve_interfaces/srv/DriveChainCmd "{command: 2}"
```

---

### Real hardware (DDSM Driver HAT on Raspberry Pi)

Ensure the Pi UART is configured (see [One-time Pi setup](#one-time-pi-setup)) and both motors are assigned unique IDs (see [Motor IDs](#motor-ids)).

**Terminal 1 — node:**
```bash
source install/setup.bash
ros2 run mserve_drivechain drivechain_node --ros-args \
  -p drive.backend:=hardware \
  -p hardware.uart_device:=/dev/serial0
```

**Terminal 2 — lifecycle + control:**
```bash
source install/setup.bash

ros2 lifecycle set /mserve_drivechain configure
ros2 lifecycle set /mserve_drivechain activate

# Connect — opens UART, pings both motors, sets speed-loop mode
ros2 service call /mserve_drivechain/drivechain_cmd mserve_interfaces/srv/DriveChainCmd "{command: 1}"

# Drive forward at 0.2 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Stop and close UART
ros2 service call /mserve_drivechain/drivechain_cmd mserve_interfaces/srv/DriveChainCmd "{command: 2}"
```

---

### Real-time monitoring (sim and hardware)

```bash
source install/setup.bash

# Wheel velocity and position feedback
ros2 topic echo /mserve_drivechain/wheel_feedback

# Connection state (idle_sim / connected_sim / idle_hw / connected_hw)
ros2 topic echo /mserve_drivechain/drive_status

# Check drive loop is ticking at the configured rate
ros2 topic hz /mserve_drivechain/wheel_feedback
```

Expected feedback at `linear.x: 0.2 m/s` (default geometry: sep=0.35 m, radius=0.08 m):
- `left_velocity` ≈ `right_velocity` ≈ **2.5 rad/s**
- No angular component → both wheels at equal speed

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
| Drive | `drive_tree.xml` | Timer tick (every 1/feedback_rate seconds) |

The drive tree short-circuits silently when not connected (`UartOpen` returns FAILURE), so cmd_vel commands are safely ignored until CONNECT has been called.

## Tests

```bash
colcon test --packages-select mserve_drivechain --event-handlers console_direct+
```

- `test_packet_codec` — CRC-8/MAXIM verification + sim-mode DriveUart round-trips

## DDSM115 protocol notes

- 115200 baud, RS-485 half-duplex on single bus
- 10-byte packets: CRC-8/MAXIM (polynomial 0x8C) over bytes [0..8], stored in byte [9]
- Speed loop (mode=2): RPM range −200 to +200
- Feedback: mode, torque, speed_rpm, position (0–32767 = 0–360°), fault_code
- Mode change quirk: byte[9] = mode value directly, not a CRC
- ID change: send 5× with 4 ms gaps, then verify with ping

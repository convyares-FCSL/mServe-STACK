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

`PublishDriveStatus` also calls `uart->board_alive()` directly (not a
blackboard key) to populate `DriveStatus.board_alive` — see
[Liveness heartbeat](#liveness-heartbeat-esp32--pi).

## Hardware: DDSM Driver HAT (A) on Raspberry Pi

The HAT sits on the 40-pin GPIO header and communicates over the Pi's **hardware UART** — no USB required.

### Physical switch: "Serial Port Control Switch"

The HAT has a small slide switch that selects which interface reaches the
onboard ESP32's UART0:

| Switch position | ESP32 UART0 connected to |
|---|---|
| **ESP32** | Pi GPIO14/15 (header pins 8/10) — required for `mserve_drivechain` |
| **USB** | The HAT's USB-C port (CH343/CH9102 USB-UART bridge → `/dev/ttyACM0` on the Pi) |

Set it to **ESP32** for normal operation. Only use **USB** for flashing
firmware or debugging directly over USB-C — in that position the GPIO header
UART is disconnected and `mserve_drivechain` will see no traffic at all.

### One-time Pi setup

```bash
sudo raspi-config
# → Interface Options → Serial Port
#   "Would you like a login shell over serial?" → No
#   "Would you like the serial port hardware enabled?" → Yes
sudo reboot
```

### UART device path — Raspberry Pi 5 vs Pi 4

> **The single most important thing to get right on a Pi 5.**

On a **Pi 4**, `/dev/serial0` symlinks to `/dev/ttyAMA0`, the PL011 UART
wired to GPIO14/15 — the commonly-documented setup works as-is.

On a **Pi 5**, `/dev/serial0` instead symlinks to `/dev/ttyAMA10`, which is
**RP1's** UART0 (`rp1/serial@30000`, MMIO `0x107d001000`) — **not connected
to the 40-pin header**. Confusingly, `pinctrl funcs 14 15` still reports
alt-function `a4` ("TXD0"/"RXD0") for GPIO14/15, which looks like
confirmation that `/dev/serial0` is correct — it isn't. That alt-function
belongs to BCM2712's *own* PL011 UART, exposed separately as
**`/dev/ttyAMA0`** (MMIO `0x1f00030000`).

| Pi model | GPIO14/15 header UART | `/dev/serial0` symlink target |
|---|---|---|
| Pi 4 | `/dev/ttyAMA0` | → `/dev/ttyAMA0` (correct) |
| Pi 5 | `/dev/ttyAMA0` | → `/dev/ttyAMA10` (RP1 UART0 — **not the header**) |

**On Pi 5, the GPIO14/15 header UART is `/dev/ttyAMA0`.** This is now the
default for `hardware.uart_device`.

If `hardware.uart_device=/dev/ttyAMA0` produces no traffic, check:
1. The "Serial Port Control Switch" is in the **ESP32** position (above).
2. Nothing else has `/dev/ttyAMA0` open (e.g. another `uart_diag.py` or
   `drivechain_node` instance — only one process can hold the port).
3. `python3 ws/src/mserve_drivechain/scripts/uart_diag.py` reports a
   heartbeat (see [Liveness heartbeat](#liveness-heartbeat-esp32--pi)).

### Motor IDs

Each DDSM115 on the RS-485 bus needs a unique ID. Factory default is ID=1 for both motors, so assign them before first use. Connect **one motor at a time** and use the SET_ID command:

```bash
# With only the left motor connected — confirm it's ID 1 (factory default)
ros2 service call /mserve_drivechain/drivechain_cmd mserve_interfaces/srv/DriveChainCmd "{command: 3, motor_id: 1, new_id: 1}"

# Swap to the right motor — reassign from ID 1 → ID 2
ros2 service call /mserve_drivechain/drivechain_cmd mserve_interfaces/srv/DriveChainCmd "{command: 3, motor_id: 1, new_id: 2}"
```

## Liveness heartbeat (ESP32 ↔ Pi)

The `ddsm_example` firmware (`drive_firmware/ddsm_active/`) sends a
periodic "I'm alive" message over UART, independent of any command/response
traffic:

```json
{"T":20020,"hb":<count>,"up":<millis_since_boot>}
```

- Sent unconditionally roughly every 1000 ms (`HEARTBEAT_INTERVAL_MS` in
  `ddsm_example.ino`, emitted by `uartHeartbeat()` in `uart_ctrl.h`).
- T-code `20020` (`FB_HEARTBEAT` in `json_cmd.h`) — distinct from
  `CMD_HEARTBEAT_TIME` (`11001`), which is the unrelated DDSM motor-stop
  watchdog (`heartbeat_ctrl()` — stops the motors if no command arrives).

On the Pi side, `DriveUart::read_line()` filters these lines out
transparently: any read (motor feedback, mode-change drain, id-check, ...)
records the heartbeat timestamp regardless of what it was actually waiting
for, so no dedicated polling is needed.

`DriveUart::board_alive(timeout_ms = 3000)`:
- always `true` in sim mode,
- `false` until the first heartbeat is seen in hardware mode,
- otherwise `true` if a heartbeat arrived within the last 3 seconds (≈3
  missed beats).

`PublishDriveStatus` reads this every drive-tree tick and publishes it as
`board_alive` on `~/drive_status`:

```bash
ros2 topic echo /mserve_drivechain/drive_status
# status: connected_hw
# battery_level: 0.0
# board_alive: true
```

This separates "the UART port is open" (`uart_connected`) from "the ESP32 on
the other end is actually running" (`board_alive`) — useful if the HAT loses
power, resets, or is unplugged while the port itself stays open.

## Parameters

| Parameter | Default | Notes |
|---|---|---|
| `drive.backend` | `"sim"` | `"sim"` = no hardware; `"hardware"` = real HAT |
| `hardware.uart_device` | `"/dev/ttyAMA0"` | UART device path (Pi 5 GPIO header UART) |
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

# Custom UART device (e.g. USB instead of the GPIO header):
./web/run_drivechain_hw.sh /dev/ttyACM0
```

The browser banner shows `[sim]` or `[hardware (/dev/ttyAMA0)]` so you can tell at a glance which mode is running.

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

> **Serial device passthrough**: `docker-compose.yml` maps `/dev/ttyAMA0` (the Pi 5 GPIO header UART, see [UART device path](#uart-device-path--raspberry-pi-5-vs-pi-4)) into the container and adds the `dialout` group. If your setup uses a different device (e.g., `/dev/ttyACM0` over USB), pass it as an argument: `./web/run_drivechain_hw.sh /dev/ttyACM0` — but also update `docker-compose.yml` devices accordingly.

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

Ensure the Pi UART is configured (see [One-time Pi setup](#one-time-pi-setup)), the "Serial Port Control Switch" is in the **ESP32** position, and both motors are assigned unique IDs (see [Motor IDs](#motor-ids)).

**Terminal 1 — node:**
```bash
source install/setup.bash
ros2 run mserve_drivechain drivechain_node --ros-args \
  -p drive.backend:=hardware \
  -p hardware.uart_device:=/dev/ttyAMA0   # default — only needed to override
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
# + board_alive (ESP32 liveness heartbeat — see Liveness heartbeat below)
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
- Liveness heartbeat: `{"T":20020,"hb":N,"up":millis}` every ~1s, unrelated
  to the DDSM packets above — see [Liveness heartbeat](#liveness-heartbeat-esp32--pi)

# mserve_drivechain

Controls one or more DDSM115 brushless motors via a Waveshare **DDSM Driver HAT (A)** mounted on a Raspberry Pi.

## Architecture

```
DrivechainNode (lifecycle)
│
├── Service  ~/connect         (std_srvs/Trigger)
│     └── connect_tree.xml — OpenUart → ConnectAllMotors (BT, under uart_mutex)
│
├── Service  ~/stop            (std_srvs/Trigger)
│     └── stop_tree.xml — StopAllMotors → CloseUart (BT, under uart_mutex)
│
├── Service  ~/drive           (interfaces/srv/Drive)
│     └── DriveCommandStore::update() — thread-safe; no BT, no uart_mutex
│
├── Service  ~/set_motor_id    (interfaces/srv/SetMotorId)
│     └── set_id_tree.xml — StopMotor → SetMotorId → PingMotor (BT, under uart_mutex)
│
├── Timer  (feedback_rate Hz, default 10 Hz)
│     └── drive_tree.xml: UartOpen → SetAllMotors → PublishMotorFeedback → PublishDriveStatus
│           SetAllMotors: reads DriveCommandStore; zeros all motors if store is stale
│
├── Publisher  ~/motor_feedback  (DriveMotorFeedback)
└── Publisher  ~/drive_status    (DriveStatus)
```

Command flow: the web UI calls `~/drive` on every joystick update, which writes motor
RPMs to `DriveCommandStore`. The drive timer reads the store on every tick. If no
command arrives within `drive.command_timeout_ms`, all motors are zeroed.

The **blackboard** is the single source of truth shared between the node and all BT nodes:

| Key | Type | Written by | Read by |
|---|---|---|---|
| `uart` | `DriveUart*` | on_configure | BT nodes |
| `drive_cmd_store` | `DriveCommandStore*` | on_configure | SetAllMotors |
| `sim_mode` | bool | load_params | BT nodes, service handlers |
| `uart_device` | string | load_params | OpenUart |
| `motor_list` | `vector<MotorDescriptor>` | load_params | ConnectAllMotors, StopAllMotors, SetAllMotors |
| `command_timeout_ms` | int | load_params | SetAllMotors |
| `feedback_rate` | double | load_params | on_activate, on_parameters |
| `uart_connected` | bool | OpenUart / CloseUart | PublishDriveStatus |
| `motor_states` | `vector<MotorState>` | SetAllMotors | PublishMotorFeedback |
| `target_motor_id` / `new_motor_id` | int | on_set_id handler | SetMotorId BT node |
| `publish_motor_feedback` | `std::function` | on_configure | PublishMotorFeedback |
| `publish_drive_status` | `std::function` | on_configure | PublishDriveStatus |

### Key source files

| File | Purpose |
|---|---|
| `include/mserve_drivechain/drivechain_node.hpp` | Node public API |
| `src/drivechain_node.cpp` | Lifecycle + BT runner + service handlers |
| `src/drivechain_params.cpp` | Parameter declaration, loading, hot-change |
| `src/drivechain_bt_nodes.cpp` | All BT node implementations |
| `src/include/drivechain_types.hpp` | `MotorFeedback`, `MotorDescriptor`, `DriveCommandStore` |
| `src/include/drivechain_uart.hpp` / `.cpp` | UART protocol layer (JSON over RS-485) |
| `src/trees/*.xml` | BT tree definitions |

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

Each DDSM115 on the RS-485 bus needs a unique ID. Factory default is ID=1 for both motors, so assign them before first use. Connect **one motor at a time** and use the `~/set_motor_id` service:

```bash
source install/setup.bash

# Node must be configured, activated, and connected first.

# Confirm the single motor is ID 1 (factory default) — change to itself is a no-op
ros2 service call /mserve_drivechain/set_motor_id interfaces/srv/SetMotorId "{motor_id: 1, new_id: 1}"

# Swap to the second motor — reassign from ID 1 → ID 2
ros2 service call /mserve_drivechain/set_motor_id interfaces/srv/SetMotorId "{motor_id: 1, new_id: 2}"
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
| `hardware.motor_count` | `2` | Number of motors (1–4) |
| `hardware.motor_ids` | `[1, 2]` | DDSM115 bus IDs |
| `hardware.motor_names` | `["left", "right"]` | Human-readable labels (used in logs) |
| `hardware.motor_signs` | `[1, 1]` | `+1` = shaft forward = robot forward; `-1` = physically reversed |
| `hardware.motor_enabled` | `[true, true]` | Per-motor enable; disabled motors are skipped |
| `drive.command_timeout_ms` | `500` | Zero all motors if no `~/drive` call within this window |
| `feedback_rate` | `10.0` | Drive loop / publish rate (Hz) |

Hardware params (`drive.backend`, `hardware.*`) can only be changed in `UNCONFIGURED` state.
`feedback_rate` and `drive.command_timeout_ms` are hot-changeable at runtime.

## Build

```bash
cd ~/ai-workspace/projects/mServe-STACK/ws
colcon build --packages-select interfaces mserve_drivechain
source install/setup.bash
```

## Web UI

A browser-based control panel is in `web/` at the repo root. It connects to ROS via rosbridge and provides lifecycle controls, motor connect/stop, a D-pad with speed sliders, and live motor feedback.

```
web/
├── drivechain.html     — drivechain control page
├── drivechain.js
├── index.html          — main debug bridge (links to drivechain page)
├── run_drivechain_hw.sh — one-command stack launcher (sim or hardware)
└── roslib.min.js
```

The web UI talks to four ROS services directly — no topic publisher, no DDS discovery delay:

| UI action | ROS service | Type |
|---|---|---|
| Connect motors | `~/connect` | `std_srvs/Trigger` |
| Stop motors | `~/stop` | `std_srvs/Trigger` |
| D-pad / joystick | `~/drive` | `interfaces/srv/Drive` |
| (Motor ID setup) | `~/set_motor_id` | `interfaces/srv/SetMotorId` |

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
| **Connect motors** | Calls `~/connect` — opens UART, pings motors, sets speed mode |
| **Stop motors** | Calls `~/stop` — zeros motors, closes UART |
| Linear / Turn rate sliders | Sets max RPM for D-pad buttons |
| ▲ ▼ ◄ ► | Hold to drive in that direction; release to stop |
| ■ | Immediate stop (zero all motors) |
| W / A / S / D or arrow keys | Same as D-pad, hold to move |
| Space | Stop |

The drive status badge updates live: `idle_sim`, `connected_sim`, `idle_hw`, `connected_hw`.

---

## Run (manual / CLI)

> **Every new terminal must source the workspace before using `ros2 service call`.**
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
ros2 service call /mserve_drivechain/connect std_srvs/srv/Trigger

# Drive motor 1 at +100 RPM, motor 2 at +100 RPM
ros2 service call /mserve_drivechain/drive interfaces/srv/Drive \
  "{motor_commands: [{motor_id: 1, rpm: 100}, {motor_id: 2, rpm: 100}]}"

# Stop and close
ros2 service call /mserve_drivechain/stop std_srvs/srv/Trigger
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
ros2 service call /mserve_drivechain/connect std_srvs/srv/Trigger

# Drive forward (both motors at 100 RPM)
ros2 service call /mserve_drivechain/drive interfaces/srv/Drive \
  "{motor_commands: [{motor_id: 1, rpm: 100}, {motor_id: 2, rpm: 100}]}"

# Stop and close UART
ros2 service call /mserve_drivechain/stop std_srvs/srv/Trigger
```

---

### Real-time monitoring (sim and hardware)

```bash
source install/setup.bash

# Per-motor feedback (mode, speed_rpm, position, fault_code, current, temperature)
ros2 topic echo /mserve_drivechain/motor_feedback

# Connection state (idle_sim / connected_sim / idle_hw / connected_hw) + board_alive
ros2 topic echo /mserve_drivechain/drive_status

# Check drive loop is ticking at the configured rate
ros2 topic hz /mserve_drivechain/motor_feedback

# Enable drive command debug logging (shows every ~/drive call)
# Launch with: --ros-args --log-level mserve_drivechain:=DEBUG
```

## Service reference

| Service | Type | Request fields | Effect |
|---|---|---|---|
| `~/connect` | `std_srvs/srv/Trigger` | — | Open UART, ping all motors, set speed mode |
| `~/stop` | `std_srvs/srv/Trigger` | — | Zero all motors, close UART |
| `~/drive` | `interfaces/srv/Drive` | `motor_commands[]` (`motor_id`, `rpm`) | Update `DriveCommandStore`; applied on next timer tick |
| `~/set_motor_id` | `interfaces/srv/SetMotorId` | `motor_id`, `new_id` | Change motor hardware ID (one motor on bus only) |

All handlers check `drive_active_` (node must be in ACTIVE state) and return
`success=false` with a message if not.

## BT trees

| Tree | File | Triggered by |
|---|---|---|
| Connect | `connect_tree.xml` | `~/connect` service call |
| Stop | `stop_tree.xml` | `~/stop` service call or on_deactivate |
| Set ID | `set_id_tree.xml` | `~/set_motor_id` service call |
| Drive | `drive_tree.xml` | Timer tick (every 1/feedback_rate seconds) |

The drive tree short-circuits silently when not connected (`UartOpen` returns FAILURE), so drive commands are safely ignored until `~/connect` has been called.

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

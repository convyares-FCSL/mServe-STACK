#!/usr/bin/env python3
"""
uart_diag.py — DDSM Driver HAT (A) / ESP32 JSON serial diagnostic

Communicates with the HAT's ESP32 using JSON commands (NOT raw DDSM115
binary). The ESP32 translates JSON → RS-485 → motors internally.

Switch position: "Serial Port Control Switch" must be set to ESP32.
  - /dev/ttyAMA0  → Pi GPIO header UART (pins 14/15) → ESP32 (switch in ESP32 position)
  - /dev/ttyACM0  → USB-C cable (CH343 USB-UART)     → ESP32 (switch in USB position)

NOTE (Raspberry Pi 5): /dev/serial0 is NOT the GPIO header UART on Pi 5 — it
symlinks to ttyAMA10 (RP1's UART0), which is not wired to the 40-pin header.
The header UART is /dev/ttyAMA0 (BCM2712's own PL011). See README.md.

The ESP32 also emits a {"T":20020,"hb":N,"up":millis} liveness heartbeat
about once a second — this script listens for one before running the
active checks below, as a quick "is the board alive at all" sanity check.

Usage:
    python3 uart_diag.py [device] [motor_ids...]

Examples:
    python3 uart_diag.py
    python3 uart_diag.py /dev/ttyAMA0 1 2
    python3 uart_diag.py /dev/ttyACM0 1
"""

import sys
import json
import time

DEVICE   = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyAMA0"
IDS      = [int(x) for x in sys.argv[2:]] if len(sys.argv) > 2 else [1, 2]
BAUD     = 115200
TIMEOUT  = 0.5  # 500 ms — generous for ESP32 processing

# ── Open port ──────────────────────────────────────────────────────────────────
import os, termios, fcntl, select

def open_port(device: str, baud: int):
    try:
        fd = os.open(device, os.O_RDWR | os.O_NOCTTY)
    except OSError as e:
        print(f"  ERROR: cannot open {device}: {e}")
        sys.exit(1)

    tty = termios.tcgetattr(fd)
    baud_const = {9600: termios.B9600, 57600: termios.B57600, 115200: termios.B115200}[baud]

    # In Python 3, ispeed=tty[4] and ospeed=tty[5]
    tty[4] = baud_const
    tty[5] = baud_const

    # 8N1, no flow control, raw mode
    tty[2] = (tty[2] & ~termios.CSIZE) | termios.CS8
    tty[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
    tty[2] |=  (termios.CLOCAL | termios.CREAD)
    tty[3] &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
    tty[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
    tty[1] &= ~termios.OPOST
    tty[6][termios.VMIN]  = 0
    tty[6][termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW, tty)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd

def send_cmd(fd, cmd: str):
    os.write(fd, (cmd + "\n").encode())

def read_line(fd, timeout: float) -> str:
    deadline = time.monotonic() + timeout
    buf = b""
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return ""
        r, _, _ = select.select([fd], [], [], remaining)
        if not r:
            return ""
        ch = os.read(fd, 1)
        if not ch:
            return ""
        if ch == b"\n":
            return buf.decode(errors="replace").strip()
        buf += ch

# ── Main ───────────────────────────────────────────────────────────────────────
print(f"\n{'='*60}")
print(f"  DDSM115 UART Diagnostic")
print(f"  Device : {DEVICE}")
print(f"  Baud   : {BAUD}")
print(f"  IDs    : {IDS}")
print(f"{'='*60}\n")

# 1. Check device exists
if not os.path.exists(DEVICE):
    print(f"  FAIL: {DEVICE} does not exist")
    print("        Check docker-compose.yml devices: mapping and raspi-config serial.")
    sys.exit(1)
print(f"  OK  : {DEVICE} exists")

# 2. Open port
fd = open_port(DEVICE, BAUD)
print(f"  OK  : opened {DEVICE} at {BAUD} baud\n")

# 3. Listen for a liveness heartbeat ({"T":20020,...}, ~1/s) — confirms the
#    board is powered, running, and the switch/wiring is correct, even
#    before sending anything.
print("  Listening for heartbeat (T:20020, up to 1.5s) ...", end="  ", flush=True)
hb_deadline = time.monotonic() + 1.5
heartbeat_seen = False
while time.monotonic() < hb_deadline:
    line = read_line(fd, hb_deadline - time.monotonic())
    if not line:
        break
    try:
        d = json.loads(line)
    except json.JSONDecodeError:
        continue
    if d.get("T") == 20020:
        heartbeat_seen = True
        print(f"OK — {line}")
        break
if not heartbeat_seen:
    print("NONE")
    print("        → board not running, wrong device, or switch in USB position")

print()

# 5. ID check — query ESP32 for any connected motor (one motor on bus)
print("  Sending ID check ({"+"T:10031} — needs single motor on bus) ...", end="  ", flush=True)
send_cmd(fd, '{"T":10031}')
line = read_line(fd, TIMEOUT)
if line:
    print(f"Response: {line}")
    try:
        d = json.loads(line)
        print(f"        ✓ ESP32 responded — motor ID={d.get('id','?')} type={d.get('typ','?')}")
    except json.JSONDecodeError:
        print(f"        → non-JSON response: {line!r}")
else:
    print("NO RESPONSE")
    print("        → check switch position (must be ESP32), motor powered, wiring")

print()

# 6. Ping each motor ID with a speed=0 command
any_found = False
for mid in IDS:
    cmd = f'{{"T":10010,"id":{mid},"cmd":0,"act":0}}'
    print(f"  Pinging motor ID {mid} (speed=0 cmd) ...", end="  ", flush=True)
    send_cmd(fd, cmd)
    # scan up to 3 lines for a matching response
    found = False
    for _ in range(3):
        resp = read_line(fd, TIMEOUT)
        if not resp:
            break
        try:
            d = json.loads(resp)
            if d.get("id") == mid:
                any_found = found = True
                print(f"Response: {resp}")
                print(f"        ✓ Motor {mid} alive — spd={d.get('spd')} rpm, "
                      f"crt={d.get('crt')} A, tep={d.get('tep')}°C, err={d.get('err')}")
                break
        except json.JSONDecodeError:
            print(f"        non-JSON: {resp!r}")
            break
    if not found:
        print("NO RESPONSE")
        print(f"        → motor ID {mid} not responding — powered? connected? correct ID?")

os.close(fd)
print()

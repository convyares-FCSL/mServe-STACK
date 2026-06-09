#!/usr/bin/env python3
"""
uart_diag.py — DDSM115 RS-485 serial diagnostic

Tries to talk to DDSM115 motors on the given UART device and prints what
it finds. Run this BEFORE the ROS node to rule out wiring / switch issues.

Usage (inside Docker container or natively on Pi):
    python3 uart_diag.py [device] [motor_ids...]

Examples:
    python3 uart_diag.py
    python3 uart_diag.py /dev/serial0
    python3 uart_diag.py /dev/serial0 1 2
    python3 uart_diag.py /dev/ttyUSB0 1
"""

import sys
import struct
import time

DEVICE   = sys.argv[1] if len(sys.argv) > 1 else "/dev/serial0"
IDS      = [int(x) for x in sys.argv[2:]] if len(sys.argv) > 2 else [1, 2]
BAUD     = 115200
TIMEOUT  = 0.05  # 50 ms receive window

# ── CRC-8/MAXIM (polynomial 0x8C, reflected) ──────────────────────────────────
def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc

# ── Packet builders ────────────────────────────────────────────────────────────
def stop_packet(motor_id: int) -> bytes:
    pkt = bytearray(10)
    pkt[0] = motor_id
    pkt[1] = 0x64   # speed command
    # bytes 2-8 = 0x00  (RPM = 0)
    pkt[9] = crc8(pkt[:9])
    return bytes(pkt)

def broadcast_id_check() -> bytes:
    # Fixed broadcast packet — use only with one motor on bus
    return bytes([0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE])

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
    termios.cfsetospeed(tty, baud_const)
    termios.cfsetispeed(tty, baud_const)

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

def send_recv(fd, pkt: bytes, timeout: float):
    os.write(fd, pkt)
    deadline = time.monotonic() + timeout
    buf = b""
    while len(buf) < 10:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        r, _, _ = select.select([fd], [], [], remaining)
        if not r:
            break
        chunk = os.read(fd, 10 - len(buf))
        if not chunk:
            break
        buf += chunk
    return buf

def parse_feedback(data: bytes):
    if len(data) < 10:
        return None
    if crc8(data[:9]) != data[9]:
        return None
    torque    = struct.unpack(">h", data[2:4])[0]
    speed_rpm = struct.unpack(">h", data[4:6])[0]
    position  = struct.unpack(">H", data[6:8])[0]
    fault     = data[8]
    return {"mode": data[1], "torque": torque, "speed_rpm": speed_rpm,
            "position": position, "fault": fault}

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

# 3. Ping each motor ID
any_found = False
for mid in IDS:
    pkt  = stop_packet(mid)
    print(f"  Pinging motor ID {mid} ...", end="  ", flush=True)
    resp = send_recv(fd, pkt, TIMEOUT)

    if not resp:
        print(f"NO RESPONSE (timeout {int(TIMEOUT*1000)} ms)")
        print(f"        → motor not powered? wrong ID? RS-485 wiring? HAT switch in ESP32 mode?")
        continue

    print(f"got {len(resp)} bytes: {resp.hex(' ').upper()}")

    if len(resp) < 10:
        print(f"        → truncated response — possible noise or baud rate mismatch")
        continue

    if resp[0] != mid:
        print(f"        → response ID {resp[0]} doesn't match expected {mid}")

    fb = parse_feedback(resp)
    if fb is None:
        print(f"        → CRC FAIL — data corruption? wrong baud rate?")
    else:
        any_found = True
        deg = fb['position'] * 360.0 / 32767.0
        print(f"        → mode={fb['mode']} rpm={fb['speed_rpm']} pos={deg:.1f}° fault={fb['fault']}")
        print(f"        ✓ Motor {mid} alive")

# 4. Broadcast ID check (only if no motors found, and only safe with 1 motor on bus)
if not any_found:
    print(f"\n  Trying broadcast ID check (safe only with 1 motor on bus) ...")
    pkt  = broadcast_id_check()
    resp = send_recv(fd, pkt, TIMEOUT)
    if not resp:
        print("  NO RESPONSE to broadcast — bus is silent")
        print("\n  Likely causes:")
        print("    1. HAT DIP switch in ESP32/USB mode — set to Pi UART / passthrough")
        print("    2. Motor(s) not powered")
        print("    3. RS-485 A/B wires swapped or not connected")
        print("    4. Wrong UART device (try /dev/ttyAMA0 instead of /dev/serial0)")
    else:
        print(f"  Got {len(resp)} bytes: {resp.hex(' ').upper()}")
        fb = parse_feedback(resp)
        if fb:
            print(f"  Motor responded with ID={resp[0]} — that's the ID to use in params")

os.close(fd)
print()

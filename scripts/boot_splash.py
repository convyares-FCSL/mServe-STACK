#!/usr/bin/env python3
"""One-shot boot splash for the ELEGOO 3.5" SPI screen — closed eyes + the
Pi's current IP address, painted directly to /dev/fb0.

Deliberately standalone: no ROS, no Docker, not even mserve_display's own
Framebuffer class. This has to run and finish *before* the Docker
container / colcon build / ROS launch even starts (that whole sequence
takes ~15-30s), so if the Pi just came up on a new/unfamiliar network,
there's still a way to read its IP off the screen immediately rather than
staring at a blank display for half a minute. mserve_display's own Face
screen overwrites this with open eyes once the real stack is up — see
systemd/mserve-drivechain.service's ExecStartPre.

Geometry (480x320, stride=480px, RGB565, no padding) is hardcoded from the
confirmed-working values logged by mserve_display's Framebuffer class on
this exact hardware/overlay (dtoverlay=tft35a,rotate=90) — this script
isn't meant to be a general-purpose framebuffer tool.
"""
import struct
import subprocess
import sys

FB_DEVICE = "/dev/fb0"
WIDTH = 480
HEIGHT = 320

COL_BG = (10, 15, 30)
COL_WHITE = (255, 255, 255)

# Minimal 5x7 font — only what "IP: 123.456.789.012" needs. Same glyph
# encoding as ws/src/mserve_display/include/mserve_display/font8x8.hpp
# (bits 4..0 per row, left to right) — kept in sync by hand since this
# script has no access to the C++ header at boot time.
FONT = {
    '0': (0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E),
    '1': (0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E),
    '2': (0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F),
    '3': (0x1E, 0x01, 0x06, 0x01, 0x01, 0x11, 0x0E),
    '4': (0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02),
    '5': (0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E),
    '6': (0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E),
    '7': (0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08),
    '8': (0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E),
    '9': (0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C),
    'I': (0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E),
    'P': (0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10),
    ':': (0x00, 0x04, 0x00, 0x00, 0x04, 0x00, 0x00),
    '.': (0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00),
    ' ': (0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
}


def rgb565(r, g, b):
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)


def get_ip():
    try:
        out = subprocess.run(
            ["hostname", "-I"], capture_output=True, text=True, timeout=5).stdout.strip()
        first = out.split()[0] if out.split() else ""
        return first or "no IP yet"
    except Exception:
        return "no IP yet"


def new_frame(bg):
    return [bg] * (WIDTH * HEIGHT)


def set_px(frame, x, y, color):
    if 0 <= x < WIDTH and 0 <= y < HEIGHT:
        frame[y * WIDTH + x] = color


def fill_rect(frame, x, y, w, h, color):
    for row in range(max(y, 0), min(y + h, HEIGHT)):
        for col in range(max(x, 0), min(x + w, WIDTH)):
            frame[row * WIDTH + col] = color


def draw_text(frame, x, y, text, color, scale=2):
    cursor = x
    for ch in text.upper():
        glyph = FONT.get(ch, FONT[' '])
        for row in range(7):
            bits = glyph[row]
            for col in range(5):
                if bits & (1 << (4 - col)):
                    fill_rect(frame, cursor + col * scale, y + row * scale, scale, scale, color)
        cursor += 6 * scale


def draw_closed_eye(frame, cx, cy, color):
    # A thin horizontal capsule — closed eyelid, same position/scale as
    # mserve_display's open-eye sclera so the handoff looks like blinking
    # open rather than a jarring layout change.
    fill_rect(frame, cx - 50, cy - 6, 100, 12, color)


def main():
    bg = rgb565(*COL_BG)
    white = rgb565(*COL_WHITE)
    frame = new_frame(bg)

    draw_closed_eye(frame, 160, 160, white)
    draw_closed_eye(frame, 320, 160, white)

    ip_text = "IP: " + get_ip()
    text_w = len(ip_text) * 6 * 2
    draw_text(frame, (WIDTH - text_w) // 2, 250, ip_text, white, scale=2)

    data = b"".join(struct.pack("<H", px) for px in frame)
    try:
        with open(FB_DEVICE, "wb") as f:
            f.write(data)
    except OSError as e:
        # Non-fatal — a missing/busy framebuffer shouldn't block boot.
        print(f"boot_splash: failed to write {FB_DEVICE}: {e}", file=sys.stderr)


if __name__ == "__main__":
    main()

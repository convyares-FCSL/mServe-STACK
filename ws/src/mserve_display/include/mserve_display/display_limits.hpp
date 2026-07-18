#pragma once

namespace mserve_display {

// Touch tap detection — see touch_input.hpp. Best-guess defaults, not yet
// tuned against real hardware; see README.md's "Known-unverified" section.
constexpr int kTapMaxMoveRaw   = 200;   // out of the 0-4095 ABS_X/Y range
constexpr int kTapMinHoldMs    = 30;
constexpr int kTapMaxHoldMs    = 800;
constexpr double kTouchPollHz  = 40.0;

// Minimum gap between two accepted taps in the Calibrate wizard — a real
// hardware test showed an accidental double-tap on the same prompt (two
// taps ~400ms apart) silently consumed two calibration steps (Up counted
// twice, Down never captured), producing a corrupted near-zero-width Y
// range. Comfortably above tap_max_hold_ms so it never rejects a
// deliberate second tap, just an accidental immediate repeat.
constexpr int kCalibTapDebounceMs = 1000;

// Touch raw coordinate range (evdev ABS_X/ABS_Y min/max, confirmed via
// evtest against the physical ADS7846 device).
constexpr int kTouchRawMin = 0;
constexpr int kTouchRawMax = 4095;

// Eye-follow smoothing/deadzone — see screens.cpp's renderFace().
constexpr double kEyeSmoothing = 0.7;
constexpr double kEyeDeadzone  = 0.08;
constexpr double kEyeDefaultMaxAngular = 1.2;  // rad/s, matches mserve_base's default max_angular_speed

// Menu/Info auto-return to Face after this long with no taps — not applied
// to Calibrate, which is a multi-step wizard the user is actively working
// through (a mid-wizard timeout would just be annoying, and the Face-eyes
// UX motivation — "don't leave a status screen up forever" — doesn't apply
// while actively calibrating).
constexpr int kMenuInfoTimeoutMs = 10000;

// Screen refresh cadences.
constexpr double kInfoRefreshHz     = 1.0;
constexpr double kStatusPublishHz   = 1.0;
constexpr double kLifecyclePollHz   = 1.0;
constexpr double kIpRefreshSec      = 15.0;

// Framebuffer geometry (post-rotation, confirmed via live color test with
// dtoverlay=tft35a,rotate=90 active — see transfer.md).
constexpr int kScreenWidth  = 480;
constexpr int kScreenHeight = 320;

}  // namespace mserve_display

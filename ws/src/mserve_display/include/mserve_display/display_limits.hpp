#pragma once

namespace mserve_display {

// Touch tap detection — see touch_input.hpp. Best-guess defaults, not yet
// tuned against real hardware; see README.md's "Known-unverified" section.
constexpr int kTapMaxMoveRaw   = 200;   // out of the 0-4095 ABS_X/Y range
constexpr int kTapMinHoldMs    = 30;
constexpr int kTapMaxHoldMs    = 800;
constexpr double kTouchPollHz  = 40.0;

// Touch raw coordinate range (evdev ABS_X/ABS_Y min/max, confirmed via
// evtest against the physical ADS7846 device).
constexpr int kTouchRawMin = 0;
constexpr int kTouchRawMax = 4095;

// Eye-follow smoothing/deadzone — see screens.cpp's renderFace().
constexpr double kEyeSmoothing = 0.7;
constexpr double kEyeDeadzone  = 0.08;
constexpr double kEyeDefaultMaxAngular = 1.2;  // rad/s, matches mserve_base's default max_angular_speed

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

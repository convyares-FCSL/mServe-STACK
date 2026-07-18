#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace mserve_display {

struct TouchCalibration
{
  int x_min = 0, x_max = 4095;
  int y_min = 0, y_max = 4095;
  bool invert_x = false, invert_y = false;
  // rotate=90 may swap the touch controller's raw axes relative to the
  // (already-rotated) framebuffer's screen X/Y — unknown until calibrated
  // live against the physical screen. See README.md.
  bool swap_xy = false;
};

struct TapEvent
{
  int screen_x = 0;
  int screen_y = 0;
  int raw_x = 0;  // logged for calibration — see DisplayNode's tap handler
  int raw_y = 0;
};

// Finds and reads the ADS7846 touch controller's evdev device. Never
// hardcodes /dev/input/eventN — the number has shifted before (confirmed
// this session) — instead matches by name via /proc/bus/input/devices at
// open() time.
class TouchInput
{
public:
  TouchInput(
    int screen_width, int screen_height, TouchCalibration calib,
    int tap_max_move_raw, int tap_min_hold_ms, int tap_max_hold_ms);

  bool open(const std::string & device_name_match = "ADS7846 Touchscreen");
  void close();
  bool isOpen() const {return fd_ >= 0;}

  // Non-blocking: drains all currently-available evdev events. Returns a
  // completed tap (press+release within the move/time thresholds), if one
  // finished during this call. Call from a timer, not a blocking loop.
  std::optional<TapEvent> poll();

  void setCalibration(const TouchCalibration & c) {calib_ = c;}

private:
  static std::optional<std::string> findEventDevicePath(const std::string & name_match);
  int rawToScreenX(int raw_x) const;
  int rawToScreenY(int raw_y) const;

  int screen_w_, screen_h_;
  TouchCalibration calib_;
  int tap_max_move_raw_;
  int tap_min_hold_ms_;
  int tap_max_hold_ms_;

  int fd_ = -1;

  bool touching_ = false;
  bool have_down_pos_ = false;  // BTN_TOUCH can arrive before the first ABS_X/Y of a press
  int down_raw_x_ = 0, down_raw_y_ = 0;
  int last_raw_x_ = 0, last_raw_y_ = 0;
  int64_t down_time_ms_ = 0;
};

}  // namespace mserve_display

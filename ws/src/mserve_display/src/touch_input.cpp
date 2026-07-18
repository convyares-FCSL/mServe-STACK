#include "mserve_display/touch_input.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <fstream>

#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>

namespace mserve_display {

namespace
{
int64_t nowMs()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}
}  // namespace

TouchInput::TouchInput(
  int screen_width, int screen_height, TouchCalibration calib,
  int tap_max_move_raw, int tap_min_hold_ms, int tap_max_hold_ms)
: screen_w_(screen_width), screen_h_(screen_height), calib_(calib),
  tap_max_move_raw_(tap_max_move_raw), tap_min_hold_ms_(tap_min_hold_ms),
  tap_max_hold_ms_(tap_max_hold_ms) {}

std::optional<std::string> TouchInput::findEventDevicePath(const std::string & name_match)
{
  std::ifstream f("/proc/bus/input/devices");
  if (!f.is_open()) {
    return std::nullopt;
  }

  std::string line;
  std::string current_block;
  auto check_block = [&](const std::string & block) -> std::optional<std::string> {
      if (block.find("N: Name=\"" + name_match) == std::string::npos) {
        return std::nullopt;
      }
      size_t h_pos = block.find("H: Handlers=");
      if (h_pos == std::string::npos) {
        return std::nullopt;
      }
      size_t ev_pos = block.find("event", h_pos);
      if (ev_pos == std::string::npos) {
        return std::nullopt;
      }
      size_t digits_start = ev_pos + 5;
      size_t digits_end = digits_start;
      while (digits_end < block.size() && std::isdigit(static_cast<unsigned char>(block[digits_end]))) {
        ++digits_end;
      }
      if (digits_end == digits_start) {
        return std::nullopt;
      }
      return "/dev/input/event" + block.substr(digits_start, digits_end - digits_start);
    };

  while (std::getline(f, line)) {
    if (line.empty()) {
      auto result = check_block(current_block);
      if (result) {
        return result;
      }
      current_block.clear();
    } else {
      current_block += line;
      current_block += '\n';
    }
  }
  return check_block(current_block);  // last block may not be followed by a blank line
}

bool TouchInput::open(const std::string & device_name_match)
{
  auto path = findEventDevicePath(device_name_match);
  if (!path) {
    std::fprintf(
      stderr, "[mserve_display] touch device \"%s\" not found in /proc/bus/input/devices\n",
      device_name_match.c_str());
    return false;
  }

  fd_ = ::open(path->c_str(), O_RDONLY | O_NONBLOCK);
  if (fd_ < 0) {
    std::fprintf(stderr, "[mserve_display] failed to open %s\n", path->c_str());
    return false;
  }

  std::fprintf(stderr, "[mserve_display] touch input: %s\n", path->c_str());
  return true;
}

void TouchInput::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

int TouchInput::rawToScreenX(int raw_x) const
{
  int lo = calib_.x_min, hi = calib_.x_max;
  if (hi == lo) {
    return 0;
  }
  double t = static_cast<double>(raw_x - lo) / (hi - lo);
  t = std::clamp(t, 0.0, 1.0);
  if (calib_.invert_x) {
    t = 1.0 - t;
  }
  return static_cast<int>(t * (screen_w_ - 1));
}

int TouchInput::rawToScreenY(int raw_y) const
{
  int lo = calib_.y_min, hi = calib_.y_max;
  if (hi == lo) {
    return 0;
  }
  double t = static_cast<double>(raw_y - lo) / (hi - lo);
  t = std::clamp(t, 0.0, 1.0);
  if (calib_.invert_y) {
    t = 1.0 - t;
  }
  return static_cast<int>(t * (screen_h_ - 1));
}

std::optional<TapEvent> TouchInput::poll()
{
  if (fd_ < 0) {
    return std::nullopt;
  }

  std::optional<TapEvent> result;
  input_event ev{};
  while (read(fd_, &ev, sizeof(ev)) == static_cast<ssize_t>(sizeof(ev))) {
    if (ev.type == EV_KEY && ev.code == BTN_TOUCH) {
      if (ev.value == 1) {
        touching_ = true;
        have_down_pos_ = false;
        down_time_ms_ = nowMs();
      } else if (ev.value == 0 && touching_) {
        touching_ = false;
        int64_t held_ms = nowMs() - down_time_ms_;
        int move = std::max(
          std::abs(last_raw_x_ - down_raw_x_), std::abs(last_raw_y_ - down_raw_y_));
        if (have_down_pos_ && move <= tap_max_move_raw_ &&
          held_ms >= tap_min_hold_ms_ && held_ms <= tap_max_hold_ms_)
        {
          TapEvent tap;
          tap.raw_x = last_raw_x_;
          tap.raw_y = last_raw_y_;
          int sx = calib_.swap_xy ? rawToScreenX(last_raw_y_) : rawToScreenX(last_raw_x_);
          int sy = calib_.swap_xy ? rawToScreenY(last_raw_x_) : rawToScreenY(last_raw_y_);
          tap.screen_x = sx;
          tap.screen_y = sy;
          result = tap;
        }
      }
    } else if (ev.type == EV_ABS && touching_) {
      if (ev.code == ABS_X) {
        last_raw_x_ = ev.value;
      } else if (ev.code == ABS_Y) {
        last_raw_y_ = ev.value;
      }
      if (!have_down_pos_) {
        down_raw_x_ = last_raw_x_;
        down_raw_y_ = last_raw_y_;
        have_down_pos_ = true;
      }
    }
    // EV_SYN and anything else: ignored, evdev doesn't require per-report
    // batching for this simple a use case.
  }
  return result;
}

}  // namespace mserve_display

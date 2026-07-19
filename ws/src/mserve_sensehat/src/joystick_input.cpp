#include "mserve_sensehat/joystick_input.hpp"

#include <cstdio>
#include <fstream>

#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>

namespace mserve_sensehat {

// Identical block-scanning approach to
// mserve_display::TouchInput::findEventDevicePath — see that file's comment
// for why /proc/bus/input/devices instead of assuming an event number.
std::optional<std::string> JoystickInput::findEventDevicePath(const std::string & name_match)
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
  return check_block(current_block);
}

bool JoystickInput::open(const std::string & device_name_match)
{
  auto path = findEventDevicePath(device_name_match);
  if (!path) {
    std::fprintf(
      stderr, "[mserve_sensehat] joystick device \"%s\" not found in /proc/bus/input/devices\n",
      device_name_match.c_str());
    return false;
  }

  fd_ = ::open(path->c_str(), O_RDONLY | O_NONBLOCK);
  if (fd_ < 0) {
    std::fprintf(stderr, "[mserve_sensehat] failed to open %s\n", path->c_str());
    return false;
  }

  std::fprintf(stderr, "[mserve_sensehat] joystick input: %s\n", path->c_str());
  return true;
}

void JoystickInput::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

std::vector<JoyEvent> JoystickInput::poll()
{
  std::vector<JoyEvent> events;
  if (fd_ < 0) {
    return events;
  }

  input_event ev{};
  while (read(fd_, &ev, sizeof(ev)) == static_cast<ssize_t>(sizeof(ev))) {
    if (ev.type != EV_KEY || ev.value == 2) {
      continue;  // ev.value == 2 is auto-repeat ("held") — not needed here
    }
    std::optional<JoyKey> key;
    switch (ev.code) {
      case KEY_UP: key = JoyKey::Up; break;
      case KEY_DOWN: key = JoyKey::Down; break;
      case KEY_LEFT: key = JoyKey::Left; break;
      case KEY_RIGHT: key = JoyKey::Right; break;
      case KEY_ENTER: key = JoyKey::Center; break;
      default: break;
    }
    if (key) {
      events.push_back({*key, ev.value == 1});
    }
  }
  return events;
}

}  // namespace mserve_sensehat

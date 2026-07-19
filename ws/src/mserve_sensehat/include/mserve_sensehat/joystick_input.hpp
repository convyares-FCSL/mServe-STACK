#pragma once

#include <optional>
#include <string>
#include <vector>

namespace mserve_sensehat {

enum class JoyKey { Up, Down, Left, Right, Center };

struct JoyEvent
{
  JoyKey key;
  bool pressed;  // true = pressed, false = released
};

// Reads the Sense HAT joystick's evdev device (reports as a keyboard: KEY_UP/
// DOWN/LEFT/RIGHT/ENTER). Device lookup mirrors
// mserve_display::TouchInput::findEventDevicePath — /dev/input/eventN
// numbering is enumeration-order-dependent, not fixed, so this finds it by
// name via /proc/bus/input/devices rather than assuming a path.
class JoystickInput
{
public:
  bool open(const std::string & device_name_match = "Raspberry Pi Sense HAT Joystick");
  void close();
  bool isOpen() const {return fd_ >= 0;}

  // Non-blocking — drains whatever's queued, returns all events seen (an
  // fast repeated press/release, e.g. "held", can produce more than one).
  std::vector<JoyEvent> poll();

private:
  static std::optional<std::string> findEventDevicePath(const std::string & name_match);

  int fd_ = -1;
};

}  // namespace mserve_sensehat

#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <interfaces/msg/motor_command.hpp>

namespace mserve_drivechain {

// ── Raw UART feedback from one DDSM115 motor (decoded from ESP32 JSON) ────────
//   spd → speed_rpm,  err → fault_code,  crt → current,  tep → temperature
struct MotorFeedback {
  int   mode{2};           // 1=current, 2=speed, 3=position
  int   speed_rpm{0};      // signed, −200..+200
  int   position{0};       // 0–32767 = 0–360° (not populated in speed-loop mode)
  int   fault_code{0};     // 0 = healthy
  float current{0.0f};     // amps
  float temperature{0.0f}; // °C
};

// ── Per-motor configuration built from ROS parameters at configure time ───────
struct MotorDescriptor {
  uint8_t     id;
  std::string name;
  int         sign;    // +1 = normal, -1 = physically reversed mounting
  bool        enabled;
};

// ── Thread-safe store for the last ~/drive service command ────────────────────
class DriveCommandStore {
public:
  void update(const std::vector<interfaces::msg::MotorCommand> & cmds)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    commands_ = cmds;
    stamp_    = std::chrono::steady_clock::now();
    valid_    = true;
  }

  std::vector<interfaces::msg::MotorCommand> latest() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    return commands_;
  }

  double age_ms() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!valid_) return 1e9;
    return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - stamp_).count();
  }

private:
  mutable std::mutex                         mutex_;
  std::vector<interfaces::msg::MotorCommand> commands_;
  std::chrono::steady_clock::time_point      stamp_;
  bool                                       valid_ = false;
};

}  // namespace mserve_drivechain

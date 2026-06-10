#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>

namespace mserve_base {

// ── Per-wheel configuration built from ROS parameters at configure time ───────
struct WheelDescriptor {
  std::string name;       // "left" / "right" — for logging only
  uint8_t      motor_id;  // mserve_drivechain motor_id this wheel maps to
};

// ── Thread-safe store for the last /cmd_vel message ────────────────────────────
class CmdVelStore {
public:
  void update(const geometry_msgs::msg::Twist & msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    twist_ = msg;
    stamp_ = std::chrono::steady_clock::now();
    valid_ = true;
  }

  geometry_msgs::msg::Twist latest() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    return twist_;
  }

  double age_ms() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!valid_) return 1e9;
    return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - stamp_).count();
  }

private:
  mutable std::mutex                    mutex_;
  geometry_msgs::msg::Twist             twist_;
  std::chrono::steady_clock::time_point stamp_;
  bool                                   valid_ = false;
};

}  // namespace mserve_base

#pragma once

#include <chrono>
#include <mutex>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mserve_drivechain {

// Owns a cmd_vel subscription and caches the latest Twist.
// Thread-safe. Uses steady_clock for age() so it works under sim time too
// (the watchdog just measures wall time since the last message, which is correct).
class CmdVelCache {
public:
  template <typename NodeT>
  CmdVelCache(NodeT & node, const std::string & topic = "cmd_vel")
  {
    sub_ = node.template create_subscription<geometry_msgs::msg::Twist>(
      topic, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        twist_ = *msg;
        stamp_ = std::chrono::steady_clock::now();
        valid_ = true;
      });
  }

  geometry_msgs::msg::Twist latest() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    return twist_;
  }

  // Milliseconds since last message. Returns a large value if no message yet.
  double age_ms() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!valid_) return 1e9;
    auto elapsed = std::chrono::steady_clock::now() - stamp_;
    return std::chrono::duration<double, std::milli>(elapsed).count();
  }

  bool valid() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    return valid_;
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  mutable std::mutex mutex_;
  geometry_msgs::msg::Twist twist_;
  std::chrono::steady_clock::time_point stamp_;
  bool valid_ = false;
};

}  // namespace mserve_drivechain

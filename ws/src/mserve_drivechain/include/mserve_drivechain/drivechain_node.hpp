#ifndef MSERVE_DRIVECHAIN_DRIVECHAIN_NODE_HPP
#define MSERVE_DRIVECHAIN_DRIVECHAIN_NODE_HPP

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <mserve_interfaces/msg/esp32_status.hpp>
#include <mserve_interfaces/msg/wheel_feedback.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "mserve_drivechain/diff_drive.hpp"

namespace mserve_drivechain {

 // ==============================================================================
// DrivechainNode: a lifecycle node that subscribes to cmd_vel, applies speed limits, and republishes safely.
// ==============================================================================

class DrivechainNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit DrivechainNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_feedback_timer();
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> & params);
  void apply_feedback_rate(double rate);

  std::string cmd_vel_safe_topic_;
  std::string wheel_feedback_topic_;
  std::string drivechain_status_topic_;
  double wheel_separation_{};
  double wheel_radius_{};
  double feedback_rate_{};

  bool active_ = false;
  WheelSpeeds last_speeds_{0.0, 0.0};

  std::unique_ptr<DiffDrive> diff_drive_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp_lifecycle::LifecyclePublisher<mserve_interfaces::msg::WheelFeedback>::SharedPtr wheel_feedback_pub_;
  rclcpp_lifecycle::LifecyclePublisher<mserve_interfaces::msg::Esp32Status>::SharedPtr drivechain_status_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace mserve_drivechain

#endif  // MSERVE_DRIVECHAIN_DRIVECHAIN_NODE_HPP

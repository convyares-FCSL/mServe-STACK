#ifndef MSERVE_BASE_BASE_NODE_HPP
#define MSERVE_BASE_BASE_NODE_HPP

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace mserve_base {

// ==============================================================================
// BaseNode: a lifecycle node that subscribes to cmd_vel, applies speed limits, and republishes safely.
// ==============================================================================

class BaseNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit BaseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> & params);

  double max_linear_speed_{};
  double max_angular_speed_{};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_safe_pub_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace mserve_base

#endif  // MSERVE_BASE_BASE_NODE_HPP

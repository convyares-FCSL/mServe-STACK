#include "mserve_base/base_node.hpp"
#include "mserve_utils/param_guard.hpp"
#include "mserve_utils/qos.hpp"
#include "mserve_utils/topics.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <algorithm>
#include <vector>

namespace mserve_base {

// ==============================================================================
// Construction
// ==============================================================================

BaseNode::BaseNode(const rclcpp::NodeOptions & options): rclcpp_lifecycle::LifecycleNode("mserve_base", options){
  // Bounded descriptors let ROS reject out-of-range values before on_parameters
  // is called, so the callback can just apply the value directly.
  this->declare_parameter("limits.max_linear_speed", 0.8, mserve_utils::bounded_double("Maximum linear speed (m/s)", 0.01, 10.0));
  this->declare_parameter("limits.max_angular_speed", 1.2, mserve_utils::bounded_double("Maximum angular speed (rad/s)", 0.01, 10.0));

  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      return this->on_parameters(params);
    });

  RCLCPP_INFO(get_logger(), "Mserve_base base_node constructed");
}

// ==============================================================================
// Lifecycle Callbacks
// ==============================================================================

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BaseNode::on_configure(const rclcpp_lifecycle::State &){
  try {
    // Load parameters, applying defaults if not set. The on_parameters callback will handle updates at runtime.
    max_linear_speed_ = mserve_utils::get_or_declare_param(
      *get_node_parameters_interface(), get_logger(),
      "limits.max_linear_speed", 0.8, "limit");
    max_angular_speed_ = mserve_utils::get_or_declare_param(
      *get_node_parameters_interface(), get_logger(),
      "limits.max_angular_speed", 1.2, "limit");

    const auto cmd_vel_topic      = mserve_topics::cmd_vel(*this);
    const auto cmd_vel_safe_topic = mserve_topics::cmd_vel_safe(*this);
    const auto cmd_qos            = mserve_qos::commands(*this);

    // Subscribing to raw cmd_vel — the node will filter and republish on cmd_vel_safe.
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic, cmd_qos,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { on_cmd_vel(msg); });

    // Publisher for the filtered cmd_vel_safe topic.
    cmd_vel_safe_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_safe_topic, cmd_qos);

    RCLCPP_INFO(get_logger(), "Configuring mserve_base: %s -> %s (max_v=%.2f, max_w=%.2f)",
      cmd_vel_topic.c_str(), cmd_vel_safe_topic.c_str(), max_linear_speed_, max_angular_speed_);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "mserve_base configuration failed: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BaseNode::on_activate(const rclcpp_lifecycle::State &){
  cmd_vel_safe_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "mserve_base activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BaseNode::on_deactivate(const rclcpp_lifecycle::State &){
  cmd_vel_safe_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "mserve_base deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BaseNode::on_cleanup(const rclcpp_lifecycle::State &){
  cmd_vel_sub_.reset();
  cmd_vel_safe_pub_.reset();
  RCLCPP_INFO(get_logger(), "mserve_base cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BaseNode::on_shutdown(const rclcpp_lifecycle::State &){
  cmd_vel_sub_.reset();
  cmd_vel_safe_pub_.reset();
  RCLCPP_INFO(get_logger(), "mserve_base shut down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// ==============================================================================
// Core Logic
// ==============================================================================

rcl_interfaces::msg::SetParametersResult BaseNode::on_parameters(const std::vector<rclcpp::Parameter> & params){
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  const auto & state = this->get_current_state().label();
  const bool reconfigure_allowed = (state == "unconfigured" || state == "configuring");

  for (const auto & p : params) {
    if (p.get_name() == "limits.max_linear_speed") {
      // Range already enforced by the parameter descriptor — just apply.
      max_linear_speed_ = p.as_double();
      RCLCPP_INFO(get_logger(), "max_linear_speed -> %.2f m/s", max_linear_speed_);

    } else if (p.get_name() == "limits.max_angular_speed") {
      max_angular_speed_ = p.as_double();
      RCLCPP_INFO(get_logger(), "max_angular_speed -> %.2f rad/s", max_angular_speed_);

    } else if (p.get_name().rfind("topic_names.", 0) == 0 || p.get_name().rfind("qos.", 0) == 0) {
      if (!reconfigure_allowed) {
        result.successful = false;
        result.reason = p.get_name() + " requires reconfigure (currently " + state + ")";
        return result;
      }
    }
  }
  return result;
}

void BaseNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg){
  if (!cmd_vel_safe_pub_ || !cmd_vel_safe_pub_->is_activated()) return;

  geometry_msgs::msg::Twist safe;
  safe.linear.x  = std::clamp(msg->linear.x,  -max_linear_speed_,  max_linear_speed_);
  safe.angular.z = std::clamp(msg->angular.z, -max_angular_speed_, max_angular_speed_);
  cmd_vel_safe_pub_->publish(safe);

}

}  // namespace mserve_base

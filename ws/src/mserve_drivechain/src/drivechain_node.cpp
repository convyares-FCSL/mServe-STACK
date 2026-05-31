#include "mserve_drivechain/drivechain_node.hpp"
#include "mserve_interfaces/msg/wheel_feedback.hpp"
#include "mserve_interfaces/msg/esp32_status.hpp"
#include "mserve_utils/param_guard.hpp"
#include "mserve_utils/qos.hpp"
#include "mserve_utils/topics.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>
#include <vector>

namespace mserve_drivechain {

// ==============================================================================
// Construction
// ==============================================================================

DrivechainNode::DrivechainNode(const rclcpp::NodeOptions & options): rclcpp_lifecycle::LifecycleNode("mserve_drivechain", options){
  // Bounded descriptors — ROS rejects out-of-range values before on_parameters.
  this->declare_parameter("hardware.wheel_separation", 0.35,
    mserve_utils::bounded_double("Wheel centre-to-centre distance (m)", 0.05, 2.0));
  this->declare_parameter("hardware.wheel_radius", 0.08,
    mserve_utils::bounded_double("Wheel radius (m)", 0.01, 0.5));
  this->declare_parameter("feedback_rate", 20.0,
    mserve_utils::bounded_double("Wheel feedback publish rate (Hz)", 1.0, 100.0));

  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      return this->on_parameters(params);
    });
}

// ==============================================================================
// Lifecycle Callbacks
// ==============================================================================

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn DrivechainNode::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    // Load parameters, applying defaults if not set. The on_parameters callback will handle updates at runtime.
    wheel_separation_ = mserve_utils::get_or_declare_param(
      *get_node_parameters_interface(), get_logger(),
      "hardware.wheel_separation", 0.35, "hardware");
    wheel_radius_ = mserve_utils::get_or_declare_param(
      *get_node_parameters_interface(), get_logger(),
      "hardware.wheel_radius", 0.08, "hardware");
    feedback_rate_ = mserve_utils::get_or_declare_param(
      *get_node_parameters_interface(), get_logger(),
      "feedback_rate", 20.0, "rate");

    diff_drive_ = std::make_unique<DiffDrive>(wheel_separation_, wheel_radius_);

    const auto cmd_vel_safe_topic      = mserve_topics::cmd_vel_safe(*this);
    const auto wheel_feedback_topic    = mserve_topics::wheel_feedback(*this);
    const auto drivechain_status_topic = mserve_topics::drivechain_status(*this);

    // Subscribing to raw cmd_vel — the node will filter and republish on cmd_vel_safe.
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_safe_topic, mserve_qos::commands(*this),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { on_cmd_vel(msg); });
    
    // Publishers for wheel feedback (e.g. encoder velocities) 
    wheel_feedback_pub_    = create_publisher<mserve_interfaces::msg::WheelFeedback>(
      wheel_feedback_topic, mserve_qos::feedback(*this));

    // Publisher for drivechain status (e.g. connection, firmware version).
    drivechain_status_pub_ = create_publisher<mserve_interfaces::msg::Esp32Status>(
      drivechain_status_topic, mserve_qos::status(*this));
    apply_feedback_rate(feedback_rate_);

    RCLCPP_INFO(get_logger(), "Configuring DrivechainNode: %s -> DDSM115 (sep=%.3fm, r=%.3fm)",
      cmd_vel_safe_topic.c_str(), wheel_separation_, wheel_radius_);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "DrivechainNode configuration failed: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn DrivechainNode::on_activate(const rclcpp_lifecycle::State &)
{
  wheel_feedback_pub_->on_activate();
  drivechain_status_pub_->on_activate();
  active_ = true;
  RCLCPP_INFO(get_logger(), "DrivechainNode activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn DrivechainNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  last_speeds_ = {0.0, 0.0};
  wheel_feedback_pub_->on_deactivate();
  drivechain_status_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "DrivechainNode deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn DrivechainNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  feedback_timer_.reset();
  cmd_vel_sub_.reset();
  wheel_feedback_pub_.reset();
  drivechain_status_pub_.reset();
  diff_drive_.reset();
  RCLCPP_INFO(get_logger(), "DrivechainNode cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn DrivechainNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  feedback_timer_.reset();
  cmd_vel_sub_.reset();
  wheel_feedback_pub_.reset();
  drivechain_status_pub_.reset();
  diff_drive_.reset();
  RCLCPP_INFO(get_logger(), "DrivechainNode shut down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// ==============================================================================
// Core Logic
// ==============================================================================

rcl_interfaces::msg::SetParametersResult DrivechainNode::on_parameters(const std::vector<rclcpp::Parameter> & params){
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : params) {
    if (p.get_name() == "feedback_rate") {
      // Range already enforced by the parameter descriptor — just apply.
      apply_feedback_rate(p.as_double());
      RCLCPP_INFO(get_logger(), "feedback_rate -> %.1f Hz", feedback_rate_);

    } else if (p.get_name() == "hardware.wheel_separation" || p.get_name() == "hardware.wheel_radius") {
      const auto & state = this->get_current_state().label();
      if (state != "unconfigured") {
        result.successful = false;
        result.reason = p.get_name() + " requires reconfigure (currently " + state + ")";
        return result;
      }

    } else if (p.get_name().rfind("topic_names.", 0) == 0 ||
               p.get_name().rfind("qos.", 0) == 0) {
      result.successful = false;
      result.reason = p.get_name() + " cannot be changed at runtime — reconfigure the node";
      return result;
    }
  }
  return result;
}

void DrivechainNode::apply_feedback_rate(double rate)
{
  feedback_rate_ = rate;
  if (feedback_timer_) feedback_timer_->cancel();
  feedback_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / feedback_rate_),
    [this]() { on_feedback_timer(); });
}

void DrivechainNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!active_) return;
  last_speeds_ = diff_drive_->compute(msg->linear.x, msg->angular.z);
  // TODO: encode last_speeds_ into DDSM115 protocol and send via transport
}

void DrivechainNode::on_feedback_timer()
{
  if (!active_) return;

  if (wheel_feedback_pub_ && wheel_feedback_pub_->is_activated()) {
    mserve_interfaces::msg::WheelFeedback feedback;
    feedback.left_velocity  = static_cast<float>(last_speeds_.left);
    feedback.right_velocity = static_cast<float>(last_speeds_.right);
    wheel_feedback_pub_->publish(feedback);
  }

  if (drivechain_status_pub_ && drivechain_status_pub_->is_activated()) {
    mserve_interfaces::msg::Esp32Status status;
    status.connected        = true;
    status.firmware_version = "ddsm115-stub-v0.1";
    drivechain_status_pub_->publish(status);
  }
}

}  // namespace mserve_drivechain

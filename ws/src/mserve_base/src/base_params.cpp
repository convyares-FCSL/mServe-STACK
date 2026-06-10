#include "mserve_base/base_node.hpp"
#include "mserve_base/base_limits.hpp"
#include "mserve_utils/utils.hpp"

#include <lifecycle_msgs/msg/state.hpp>

namespace mserve_base {

void BaseNode::declare_params()
{
  this->declare_parameter("limits.max_linear_speed", 0.8,
    mserve_utils::make_double_range_descriptor(
      "Maximum linear speed (m/s)", kLinearSpeedMin, kLinearSpeedMax));
  this->declare_parameter("limits.max_angular_speed", 1.2,
    mserve_utils::make_double_range_descriptor(
      "Maximum angular speed (rad/s)", kAngularSpeedMin, kAngularSpeedMax));

  this->declare_parameter("geometry.wheel_separation", 0.35,
    mserve_utils::make_double_range_descriptor(
      "Track width between wheel centers (m)", kWheelSeparationMin, kWheelSeparationMax));
  this->declare_parameter("geometry.wheel_radius", 0.08,
    mserve_utils::make_double_range_descriptor(
      "Wheel radius (m)", kWheelRadiusMin, kWheelRadiusMax));
  this->declare_parameter("geometry.gear_ratio", 1.0,
    mserve_utils::make_double_range_descriptor(
      "Motor revolutions per wheel revolution", kGearRatioMin, kGearRatioMax));

  this->declare_parameter<int64_t>("motor_ids.left", 2,
    mserve_utils::make_int_range_descriptor(
      "mserve_drivechain motor ID driving the left wheel", kMotorIdMin, kMotorIdMax));
  this->declare_parameter<int64_t>("motor_ids.right", 1,
    mserve_utils::make_int_range_descriptor(
      "mserve_drivechain motor ID driving the right wheel", kMotorIdMin, kMotorIdMax));

  this->declare_parameter<int64_t>("drive.cmd_vel_timeout_ms", 500,
    mserve_utils::make_int_range_descriptor(
      "Send a zero drive command after this many ms with no /cmd_vel message.",
      kCmdVelTimeoutMin, kCmdVelTimeoutMax));

  this->declare_parameter("feedback_rate", 10.0,
    mserve_utils::make_double_range_descriptor(
      "Drive loop tick rate (Hz)", kFeedbackRateMin, kFeedbackRateMax));
}

void BaseNode::load_params()
{
  blackboard_->set("max_linear_speed",  get_parameter("limits.max_linear_speed").as_double());
  blackboard_->set("max_angular_speed", get_parameter("limits.max_angular_speed").as_double());

  blackboard_->set("wheel_separation", get_parameter("geometry.wheel_separation").as_double());
  blackboard_->set("wheel_radius",     get_parameter("geometry.wheel_radius").as_double());
  blackboard_->set("gear_ratio",       get_parameter("geometry.gear_ratio").as_double());

  blackboard_->set("left_motor_id",  static_cast<int>(get_parameter("motor_ids.left").as_int()));
  blackboard_->set("right_motor_id", static_cast<int>(get_parameter("motor_ids.right").as_int()));

  blackboard_->set("cmd_vel_timeout_ms", static_cast<int>(get_parameter("drive.cmd_vel_timeout_ms").as_int()));
  blackboard_->set("feedback_rate",      get_parameter("feedback_rate").as_double());
}

rcl_interfaces::msg::SetParametersResult BaseNode::on_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  const bool is_unconfigured =
    get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;

  for (const auto & p : params) {
    const auto & name = p.get_name();

    // Geometry / motor mapping locked to UNCONFIGURED — drive_tree reads these
    // once at configure time and the kinematics math assumes they're stable.
    const bool is_geometry_param =
      name == "geometry.wheel_separation" ||
      name == "geometry.wheel_radius"     ||
      name == "geometry.gear_ratio"       ||
      name == "motor_ids.left"            ||
      name == "motor_ids.right";

    if (is_geometry_param && !is_unconfigured) {
      result.successful = false;
      result.reason = name + " can only be changed in UNCONFIGURED state";
      return result;
    }

    if ((name.rfind("topic_names.", 0) == 0 || name.rfind("qos.", 0) == 0) && !is_unconfigured) {
      result.successful = false;
      result.reason = name + " requires reconfigure (currently not unconfigured)";
      return result;
    }

    // Hot-changeable params
    if (name == "limits.max_linear_speed" && blackboard_) {
      blackboard_->set("max_linear_speed", p.as_double());
    }
    if (name == "limits.max_angular_speed" && blackboard_) {
      blackboard_->set("max_angular_speed", p.as_double());
    }
    if (name == "drive.cmd_vel_timeout_ms" && blackboard_) {
      blackboard_->set("cmd_vel_timeout_ms", static_cast<int>(p.as_int()));
    }
    if (name == "feedback_rate" && blackboard_) {
      const double rate = p.as_double();
      blackboard_->set("feedback_rate", rate);
      if (drive_timer_) {
        drive_timer_->cancel();
        drive_timer_ = create_wall_timer(
          std::chrono::duration<double>(1.0 / rate),
          [this]() { tick_drive_tree(); });
      }
    }
  }
  return result;
}

}  // namespace mserve_base

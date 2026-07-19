#include <mserve_utils/utils.hpp>

#include "mserve_joystick/joystick_limits.hpp"
#include "mserve_joystick/joystick_node.hpp"

namespace mserve_joystick {

void JoystickNode::loadParams()
{
  auto & params = *get_node_parameters_interface();
  auto logger = get_logger();

  joy_topic_ = mserve_utils::get_or_declare_param(
    params, logger, "joy_topic", std::string("/joy"), "joy topic");
  controller_profile_ = mserve_utils::get_or_declare_param(
    params, logger, "controller_profile", std::string("pi_controller"), "controller profile");

  linear_axis_name_ = mserve_utils::get_or_declare_param(
    params, logger, "drive.linear_axis", std::string("left_stick_y"), "linear axis name");
  angular_axis_name_ = mserve_utils::get_or_declare_param(
    params, logger, "drive.angular_axis", std::string("left_stick_x"), "angular axis name");
  deadzone_ = mserve_utils::get_or_declare_param(
    params, logger, "drive.deadzone", kDeadzone, "deadzone");

  speed_scale_ = mserve_utils::get_or_declare_param(
    params, logger, "speed.initial", kSpeedInitial, "speed initial");
  speed_min_ = mserve_utils::get_or_declare_param(
    params, logger, "speed.min", kSpeedMin, "speed min");
  speed_max_ = mserve_utils::get_or_declare_param(
    params, logger, "speed.max", kSpeedMax, "speed max");
  speed_step_ = mserve_utils::get_or_declare_param(
    params, logger, "speed.step", kSpeedStep, "speed step");

  angular_scale_ = mserve_utils::get_or_declare_param(
    params, logger, "angular.initial", kAngularInitial, "angular initial");
  angular_min_ = mserve_utils::get_or_declare_param(
    params, logger, "angular.min", kAngularMin, "angular min");
  angular_max_ = mserve_utils::get_or_declare_param(
    params, logger, "angular.max", kAngularMax, "angular max");
  angular_step_ = mserve_utils::get_or_declare_param(
    params, logger, "angular.step", kAngularStep, "angular step");

  status_publish_hz_ = mserve_utils::get_or_declare_param(
    params, logger, "status_publish_hz", 2.0, "status publish rate");

  topic_cmd_vel_ = mserve_utils::get_or_declare_param(
    params, logger, "topic_names.cmd_vel", std::string("/cmd_vel"), "cmd_vel topic");
  service_connect_ = mserve_utils::get_or_declare_param(
    params, logger, "service_names.connect", std::string("/mserve_drivechain/connect"),
    "connect service");
  service_display_set_mode_ = mserve_utils::get_or_declare_param(
    params, logger, "service_names.display_set_mode",
    std::string("/mserve_display/set_display_mode"), "display set_mode service");
}

// The selected controller_profiles.<controller_profile_> entry (from
// joystick_params.yaml) and button_actions (from mserve_params.yaml) both
// have arbitrary, controller/mapping-defined keys — neither is a fixed set
// known at compile time — so they're read straight out of the raw parameter
// overrides rather than via individual get_or_declare_param calls. The
// profile is index -> name (see joystick_params.yaml's header comment for
// why); button_actions is name -> action. "not_used" entries in the profile
// are declared HID slots with no real control — skipped for the internal
// name->index maps, but still declared below so they're visible (as
// "not_used") in the Parameters panel like every other key here.
//
// Each matched override is also explicitly declare_parameter'd — reading
// via get_parameter_overrides() alone (as above) makes the value usable
// internally, but does NOT register it with rclcpp's parameter service
// interface, so without this it silently wouldn't show up in `ros2 param
// list`, ros2 param get/set, or this page's Parameters panel (which all go
// through that interface, not the raw overrides map).
void JoystickNode::loadJoystickMap()
{
  auto & params = *get_node_parameters_interface();
  auto overrides = params.get_parameter_overrides();

  const std::string axes_prefix = "controller_profiles." + controller_profile_ + ".axes.";
  const std::string buttons_prefix = "controller_profiles." + controller_profile_ + ".buttons.";
  const std::string actions_prefix = "button_actions.";

  for (const auto & [key, value] : overrides) {
    if (value.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      continue;
    }
    const std::string & name = value.get<std::string>();
    const bool is_axis = key.rfind(axes_prefix, 0) == 0;
    const bool is_button = key.rfind(buttons_prefix, 0) == 0;
    const bool is_action = key.rfind(actions_prefix, 0) == 0;
    if (!is_axis && !is_button && !is_action) {
      continue;
    }

    params.declare_parameter(key, value);

    if (is_axis && name != "not_used") {
      axis_name_to_index_[name] = std::stoi(key.substr(axes_prefix.size()));
    } else if (is_button && name != "not_used") {
      button_name_to_index_[name] = std::stoi(key.substr(buttons_prefix.size()));
    } else if (is_action) {
      button_actions_[key.substr(actions_prefix.size())] = name;
    }
  }

  if (axis_name_to_index_.empty() && button_name_to_index_.empty()) {
    RCLCPP_ERROR(
      get_logger(), "controller_profiles.%s not found in joystick_params.yaml — check "
      "mserve_joystick.controller_profile in mserve_params.yaml", controller_profile_.c_str());
  }

  RCLCPP_INFO(
    get_logger(), "loaded controller profile '%s': %zu axes, %zu buttons; %zu button_actions",
    controller_profile_.c_str(), axis_name_to_index_.size(), button_name_to_index_.size(),
    button_actions_.size());
}

}  // namespace mserve_joystick

#include "mserve_drivechain/drivechain_node.hpp"
#include "mserve_drivechain/drivechain_limits.hpp"
#include "include/drivechain_types.hpp"
#include "mserve_utils/utils.hpp"

#include <lifecycle_msgs/msg/state.hpp>

namespace mserve_drivechain {

void DrivechainNode::declare_params()
{
  this->declare_parameter<std::string>("drive.backend", "sim");

  this->declare_parameter<std::string>("hardware.uart_device", "/dev/ttyAMA0");

  this->declare_parameter<int64_t>("hardware.motor_count", 2,
    mserve_utils::make_int_range_descriptor("Number of motors (1–4)", kMotorCountMin, kMotorCountMax));

  this->declare_parameter<std::vector<int64_t>>("hardware.motor_ids", {1, 2});
  this->declare_parameter<std::vector<std::string>>("hardware.motor_names", {"left", "right"});
  // +1 = motor shaft direction matches robot forward; -1 = physically reversed
  this->declare_parameter<std::vector<int64_t>>("hardware.motor_signs", {1, 1});
  this->declare_parameter<std::vector<bool>>("hardware.motor_enabled", {true, true});

  // DDSM115 'act' ramp byte (0-255); 0 = instant, higher = slower ramp to target speed.
  // Single value applied to all motors.
  this->declare_parameter<int64_t>("hardware.motor_accel", 5,
    mserve_utils::make_int_range_descriptor("DDSM115 acceleration ramp byte (0-255)", kMotorAccelMin, kMotorAccelMax));

  this->declare_parameter<int64_t>("drive.command_timeout_ms", 500,
    mserve_utils::make_int_range_descriptor(
      "Zero motors after this many ms with no motor_commands message.",
      kCommandTimeoutMin, kCommandTimeoutMax));

  this->declare_parameter("feedback_rate", 10.0,
    mserve_utils::make_double_range_descriptor("Drive loop publish rate (Hz)", kFeedbackRateMin, kFeedbackRateMax));
}

void DrivechainNode::load_params()
{
  const std::string backend = get_parameter("drive.backend").as_string();
  const bool sim_mode = (backend != "hardware");

  blackboard_->set("sim_mode",           sim_mode);
  blackboard_->set("uart_device",        get_parameter("hardware.uart_device").as_string());
  blackboard_->set("command_timeout_ms", static_cast<int>(get_parameter("drive.command_timeout_ms").as_int()));
  blackboard_->set("feedback_rate",      get_parameter("feedback_rate").as_double());

  const int   count   = static_cast<int>(get_parameter("hardware.motor_count").as_int());
  const auto  ids     = get_parameter("hardware.motor_ids").as_integer_array();
  const auto  names   = get_parameter("hardware.motor_names").as_string_array();
  const auto  signs   = get_parameter("hardware.motor_signs").as_integer_array();
  const auto  enabled = get_parameter("hardware.motor_enabled").as_bool_array();

  std::vector<MotorDescriptor> motors;
  for (int i = 0; i < count && i < static_cast<int>(ids.size()); ++i) {
    MotorDescriptor m;
    m.id      = static_cast<uint8_t>(ids[i]);
    m.name    = (i < static_cast<int>(names.size()))   ? names[i]           : "motor_" + std::to_string(i + 1);
    m.sign    = (i < static_cast<int>(signs.size()))   ? (signs[i] < 0 ? -1 : 1) : 1;
    m.enabled = (i < static_cast<int>(enabled.size())) ? enabled[i]         : true;
    motors.push_back(m);
  }
  blackboard_->set("motor_list", motors);
  blackboard_->set("motor_accel", static_cast<int>(get_parameter("hardware.motor_accel").as_int()));
}

rcl_interfaces::msg::SetParametersResult DrivechainNode::on_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  const bool is_unconfigured =
    get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;

  for (const auto & p : params) {
    const auto & name = p.get_name();

    // Hardware params locked to UNCONFIGURED
    const bool is_hw_param =
      name == "drive.backend"             ||
      name == "hardware.uart_device"      ||
      name == "hardware.motor_count"      ||
      name == "hardware.motor_ids"        ||
      name == "hardware.motor_names"      ||
      name == "hardware.motor_signs"      ||
      name == "hardware.motor_accel"      ||
      name == "hardware.motor_enabled";

    if (is_hw_param && !is_unconfigured) {
      result.successful = false;
      result.reason = name + " can only be changed in UNCONFIGURED state";
      return result;
    }

    // Validate motor_signs elements are ±1
    if (name == "hardware.motor_signs") {
      for (const auto & s : p.as_integer_array()) {
        if (s != 1 && s != -1) {
          result.successful = false;
          result.reason = "hardware.motor_signs elements must be +1 or -1";
          return result;
        }
      }
    }

    // Hot-changeable params
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
    if (name == "drive.command_timeout_ms" && blackboard_) {
      blackboard_->set("command_timeout_ms", static_cast<int>(p.as_int()));
    }
  }
  return result;
}

}  // namespace mserve_drivechain

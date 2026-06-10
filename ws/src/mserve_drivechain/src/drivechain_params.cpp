#include "mserve_drivechain/drivechain_node.hpp"
#include "mserve_drivechain/drivechain_limits.hpp"
#include "mserve_utils/utils.hpp"

#include <lifecycle_msgs/msg/state.hpp>

namespace mserve_drivechain {

void DrivechainNode::declare_params()
{
  // Backend switch: "sim" or "hardware".
  // Default is "sim" so the node works with Gazebo / without the HAT attached.
  this->declare_parameter<std::string>("drive.backend", "sim");

  // Hardware UART device — only used when backend="hardware".
  // /dev/ttyAMA0 is the GPIO14/15 header UART on Raspberry Pi 5 (BCM2712's
  // own PL011). /dev/serial0 is NOT this UART on Pi 5 — see README.md.
  this->declare_parameter<std::string>("hardware.uart_device", "/dev/ttyAMA0");

  const auto motor_id_desc = mserve_utils::make_int_range_descriptor(
    "DDSM115 motor hardware ID (1–253)", kMotorIdMin, kMotorIdMax);
  this->declare_parameter("hardware.left_motor_id",  1, motor_id_desc);
  this->declare_parameter("hardware.right_motor_id", 2, motor_id_desc);

  this->declare_parameter("hardware.wheel_separation", 0.35,
    mserve_utils::make_double_range_descriptor("Wheel centre-to-centre distance (m)", kWheelSepMin, kWheelSepMax));
  this->declare_parameter("hardware.wheel_radius", 0.08,
    mserve_utils::make_double_range_descriptor("Wheel radius (m)", kWheelRadiusMin, kWheelRadiusMax));

  const auto rpm_desc = mserve_utils::make_int_range_descriptor(
    "Maximum wheel speed (RPM). DDSM115 ceiling is 200.", 1, kMaxRpm);
  this->declare_parameter("drive.max_rpm", 200, rpm_desc);

  const auto timeout_desc = mserve_utils::make_int_range_descriptor(
    "cmd_vel watchdog — zero motors after this many ms with no command.",
    kCmdVelTimeoutMin, kCmdVelTimeoutMax);
  this->declare_parameter("drive.cmd_vel_timeout_ms", 500, timeout_desc);

  this->declare_parameter("feedback_rate", 10.0,
    mserve_utils::make_double_range_descriptor("Drive loop publish rate (Hz)", kFeedbackRateMin, kFeedbackRateMax));
}

void DrivechainNode::load_params()
{
  // Everything goes to the blackboard. BT nodes read directly from there.
  const std::string backend = get_parameter("drive.backend").as_string();
  const bool sim_mode = (backend != "hardware");

  blackboard_->set("sim_mode",           sim_mode);
  blackboard_->set("uart_device",        get_parameter("hardware.uart_device").as_string());
  blackboard_->set("left_motor_id",      static_cast<int>(get_parameter("hardware.left_motor_id").as_int()));
  blackboard_->set("right_motor_id",     static_cast<int>(get_parameter("hardware.right_motor_id").as_int()));
  blackboard_->set("wheel_separation",   get_parameter("hardware.wheel_separation").as_double());
  blackboard_->set("wheel_radius",       get_parameter("hardware.wheel_radius").as_double());
  blackboard_->set("max_rpm",            static_cast<int>(get_parameter("drive.max_rpm").as_int()));
  blackboard_->set("cmd_vel_timeout_ms", static_cast<int>(get_parameter("drive.cmd_vel_timeout_ms").as_int()));
  blackboard_->set("feedback_rate",      get_parameter("feedback_rate").as_double());
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
    const bool is_hw_param =
      name == "drive.backend"              ||
      name == "hardware.uart_device"       ||
      name == "hardware.left_motor_id"     ||
      name == "hardware.right_motor_id"    ||
      name == "hardware.wheel_separation"  ||
      name == "hardware.wheel_radius"      ||
      name == "drive.max_rpm";

    if (is_hw_param && !is_unconfigured) {
      result.successful = false;
      result.reason = name + " can only be changed in UNCONFIGURED state";
      return result;
    }

    // Hot-changeable: write to blackboard immediately.
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
    if (name == "drive.cmd_vel_timeout_ms" && blackboard_) {
      blackboard_->set("cmd_vel_timeout_ms", static_cast<int>(p.as_int()));
    }
  }
  return result;
}

}  // namespace mserve_drivechain

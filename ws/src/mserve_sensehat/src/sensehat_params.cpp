#include <mserve_utils/utils.hpp>

#include "mserve_sensehat/sensehat_node.hpp"

namespace mserve_sensehat {

void SensehatNode::declareParams() {}

void SensehatNode::loadParams()
{
  auto & params = *get_node_parameters_interface();
  auto logger = get_logger();

  imu_frame_id_ = mserve_utils::get_or_declare_param(
    params, logger, "imu_frame_id", std::string("sensehat_link"), "IMU frame_id");
  imu_publish_hz_ = mserve_utils::get_or_declare_param(
    params, logger, "imu_publish_hz", 20.0, "IMU publish rate");
  imu_settings_dir_ = mserve_utils::get_or_declare_param(
    params, logger, "imu_settings_dir", std::string("/tmp/mserve_sensehat"),
    "RTIMULib settings cache directory");
  joystick_poll_hz_ = mserve_utils::get_or_declare_param(
    params, logger, "joystick_poll_hz", 20.0, "joystick poll rate");
  joystick_device_name_match_ = mserve_utils::get_or_declare_param(
    params, logger, "joystick_device_name_match",
    std::string("Raspberry Pi Sense HAT Joystick"), "joystick device name");
  status_publish_hz_ = mserve_utils::get_or_declare_param(
    params, logger, "status_publish_hz", 5.0,
    "~/status (SensehatStatus) publish rate — web/sensehat.html");

  topic_imu_ = mserve_utils::get_or_declare_param(
    params, logger, "topic_names.imu", std::string("/mserve_sensehat/imu"), "imu topic");
  topic_status_ = mserve_utils::get_or_declare_param(
    params, logger, "topic_names.status", std::string("/mserve_sensehat/status"),
    "status topic");
  topic_drivechain_status_ = mserve_utils::get_or_declare_param(
    params, logger, "topic_names.drivechain_status",
    std::string("/mserve_drivechain/drive_status"), "drivechain_status topic");
  service_connect_ = mserve_utils::get_or_declare_param(
    params, logger, "service_names.connect", std::string("/mserve_drivechain/connect"),
    "connect service");
}

// button_actions has arbitrary keys (up/down/left/right/center), not a
// fixed set known at compile time, so it's read straight out of the raw
// parameter overrides — same reasoning and pattern as
// mserve_joystick::JoystickNode::loadJoystickMap's button_actions handling.
// Each matched override is also explicitly declare_parameter'd so it shows
// up in `ros2 param list`/get/set and the web UI's Parameters panel, not
// just internally.
void SensehatNode::loadButtonActions()
{
  auto & params = *get_node_parameters_interface();
  auto overrides = params.get_parameter_overrides();

  const std::string prefix = "button_actions.";
  for (const auto & [key, value] : overrides) {
    if (value.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      continue;
    }
    if (key.rfind(prefix, 0) != 0) {
      continue;
    }
    params.declare_parameter(key, value);
    button_actions_[key.substr(prefix.size())] = value.get<std::string>();
  }

  RCLCPP_INFO(get_logger(), "loaded %zu button_actions", button_actions_.size());
}

}  // namespace mserve_sensehat

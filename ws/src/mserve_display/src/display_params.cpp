#include <mserve_utils/utils.hpp>

#include "mserve_display/display_limits.hpp"
#include "mserve_display/display_node.hpp"
#include "mserve_display/framebuffer.hpp"

namespace mserve_display {

void DisplayNode::declareParams()
{
  // Bool params have no get_or_declare_param overload — declared directly.
  declare_parameter<bool>("fb_flip_180", false);
  declare_parameter<bool>("touch_calib.invert_x", false);
  declare_parameter<bool>("touch_calib.invert_y", false);
  declare_parameter<bool>("touch_calib.swap_xy", false);
}

void DisplayNode::loadParams()
{
  auto & params = *get_node_parameters_interface();
  auto logger = get_logger();

  fb_device_ = mserve_utils::get_or_declare_param(
    params, logger, "fb_device", std::string(""),
    "framebuffer device (empty = auto-detect the ELEGOO panel by driver name)");
  if (fb_device_.empty()) {
    fb_device_ = resolveFramebufferDevice();
    RCLCPP_INFO(logger, "fb_device auto-detected: %s", fb_device_.c_str());
  }
  fb_flip_180_ = get_parameter("fb_flip_180").as_bool();
  touch_device_name_match_ = mserve_utils::get_or_declare_param(
    params, logger, "touch.device_name_match", std::string("ADS7846 Touchscreen"),
    "touch device name");
  touch_poll_hz_ = mserve_utils::get_or_declare_param(
    params, logger, "touch.poll_rate_hz", kTouchPollHz, "touch poll rate");
  tap_max_move_raw_ = mserve_utils::get_or_declare_param(
    params, logger, "touch.tap_max_move", kTapMaxMoveRaw, "tap max move (raw units)");
  tap_min_hold_ms_ = mserve_utils::get_or_declare_param(
    params, logger, "touch.tap_min_hold_ms", kTapMinHoldMs, "tap min hold (ms)");
  tap_max_hold_ms_ = mserve_utils::get_or_declare_param(
    params, logger, "touch.tap_max_hold_ms", kTapMaxHoldMs, "tap max hold (ms)");

  touch_calib_.x_min = mserve_utils::get_or_declare_param(
    params, logger, "touch_calib.x_min", kTouchRawMin, "touch calib x_min");
  touch_calib_.x_max = mserve_utils::get_or_declare_param(
    params, logger, "touch_calib.x_max", kTouchRawMax, "touch calib x_max");
  touch_calib_.y_min = mserve_utils::get_or_declare_param(
    params, logger, "touch_calib.y_min", kTouchRawMin, "touch calib y_min");
  touch_calib_.y_max = mserve_utils::get_or_declare_param(
    params, logger, "touch_calib.y_max", kTouchRawMax, "touch calib y_max");
  touch_calib_.invert_x = get_parameter("touch_calib.invert_x").as_bool();
  touch_calib_.invert_y = get_parameter("touch_calib.invert_y").as_bool();
  touch_calib_.swap_xy = get_parameter("touch_calib.swap_xy").as_bool();

  eye_smoothing_ = mserve_utils::get_or_declare_param(
    params, logger, "eye_follow.smoothing", kEyeSmoothing, "eye smoothing");
  eye_deadzone_ = mserve_utils::get_or_declare_param(
    params, logger, "eye_follow.deadzone", kEyeDeadzone, "eye deadzone");
  eye_max_angular_ = mserve_utils::get_or_declare_param(
    params, logger, "eye_follow.max_angular_for_full_deflection", kEyeDefaultMaxAngular,
    "eye max angular");

  info_refresh_hz_ = mserve_utils::get_or_declare_param(
    params, logger, "info_refresh_hz", kInfoRefreshHz, "info refresh rate");
  status_publish_hz_ = mserve_utils::get_or_declare_param(
    params, logger, "status_publish_hz", kStatusPublishHz, "status publish rate");
  lifecycle_poll_hz_ = mserve_utils::get_or_declare_param(
    params, logger, "lifecycle_poll_hz", kLifecyclePollHz, "lifecycle poll rate");
  ip_refresh_sec_ = mserve_utils::get_or_declare_param(
    params, logger, "ip_refresh_sec", kIpRefreshSec, "IP refresh interval");

  topic_cmd_vel_safe_ = mserve_utils::get_or_declare_param(
    params, logger, "topic_names.cmd_vel_safe", std::string("/mserve/cmd_vel_safe"),
    "cmd_vel_safe topic");
  topic_motor_feedback_ = mserve_utils::get_or_declare_param(
    params, logger, "topic_names.motor_feedback",
    std::string("/mserve_drivechain/motor_feedback"), "motor_feedback topic");
  topic_drivechain_status_ = mserve_utils::get_or_declare_param(
    params, logger, "topic_names.drivechain_status",
    std::string("/mserve_drivechain/drive_status"), "drivechain_status topic");
  topic_base_status_ = mserve_utils::get_or_declare_param(
    params, logger, "topic_names.base_status", std::string("/mserve_base/base_status"),
    "base_status topic");

  service_connect_ = mserve_utils::get_or_declare_param(
    params, logger, "service_names.connect", std::string("/mserve_drivechain/connect"),
    "connect service");
  service_drivechain_get_state_ = mserve_utils::get_or_declare_param(
    params, logger, "service_names.drivechain_get_state",
    std::string("/mserve_drivechain/get_state"), "drivechain get_state service");
  service_base_get_state_ = mserve_utils::get_or_declare_param(
    params, logger, "service_names.base_get_state", std::string("/mserve_base/get_state"),
    "base get_state service");
}

}  // namespace mserve_display

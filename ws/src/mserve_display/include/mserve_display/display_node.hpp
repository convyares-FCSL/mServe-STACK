#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <interfaces/msg/display_status.hpp>
#include <interfaces/msg/drive_motor_feedback.hpp>
#include <interfaces/msg/drive_status.hpp>
#include <interfaces/srv/set_display_mode.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mserve_display/framebuffer.hpp"
#include "mserve_display/screens.hpp"
#include "mserve_display/touch_input.hpp"

namespace mserve_display {

// Plain rclcpp::Node, not lifecycle-managed — see README.md for why (no
// hardware "connect" step worth gating, same precedent as
// robot_state_publisher).
class DisplayNode : public rclcpp::Node
{
public:
  explicit DisplayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareParams();
  void loadParams();

  void onCmdVel(geometry_msgs::msg::Twist::SharedPtr msg);
  void onMotorFeedback(interfaces::msg::DriveMotorFeedback::SharedPtr msg);
  void onDrivechainStatus(interfaces::msg::DriveStatus::SharedPtr msg);
  void onBaseStatus(interfaces::msg::DriveStatus::SharedPtr msg);

  void onTouchPollTimer();
  void onTap(const TapEvent & tap);
  void onCalibrateTap(const TapEvent & tap);
  void applyCalibration();
  void onInfoRefreshTimer();
  void onLifecyclePollTimer();
  void onStatusPublishTimer();
  void onIpRefreshTimer();
  void onIdleTimeoutTimer();

  void onSetDisplayMode(
    interfaces::srv::SetDisplayMode::Request::SharedPtr request,
    interfaces::srv::SetDisplayMode::Response::SharedPtr response);

  void callConnect();
  void pollLifecycleState(
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client, std::string * state_out);

  void setScreen(Screen s);
  void requestRedraw();
  void publishStatus();
  static std::string screenName(Screen s);
  static std::optional<Screen> screenFromName(const std::string & name);
  std::string getIpAddress() const;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<interfaces::msg::DriveMotorFeedback>::SharedPtr motor_feedback_sub_;
  rclcpp::Subscription<interfaces::msg::DriveStatus>::SharedPtr drivechain_status_sub_;
  rclcpp::Subscription<interfaces::msg::DriveStatus>::SharedPtr base_status_sub_;

  // Clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr connect_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr drivechain_get_state_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr base_get_state_client_;

  // Server + publisher
  rclcpp::Service<interfaces::srv::SetDisplayMode>::SharedPtr set_mode_srv_;
  rclcpp::Publisher<interfaces::msg::DisplayStatus>::SharedPtr status_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr touch_poll_timer_;
  rclcpp::TimerBase::SharedPtr info_refresh_timer_;
  rclcpp::TimerBase::SharedPtr lifecycle_poll_timer_;
  rclcpp::TimerBase::SharedPtr status_pub_timer_;
  rclcpp::TimerBase::SharedPtr ip_refresh_timer_;
  rclcpp::TimerBase::SharedPtr idle_timeout_timer_;

  std::unique_ptr<Framebuffer> framebuffer_;
  std::unique_ptr<TouchInput> touch_;
  bool touch_available_ = false;
  bool framebuffer_available_ = false;

  Screen current_screen_ = Screen::Face;
  DisplayState state_;
  std::array<Rect, 4> menu_button_rects_{};
  double last_rendered_eye_direction_ = 0.0;
  int64_t last_calib_tap_ms_ = 0;  // debounce — see kCalibTapDebounceMs
  int64_t last_screen_activity_ms_ = 0;  // Menu/Info auto-return — see kMenuInfoTimeoutMs

  double eye_max_angular_ = 1.2;

  // Params (loaded in loadParams())
  std::string fb_device_;
  bool fb_flip_180_ = false;
  std::string touch_device_name_match_;
  double touch_poll_hz_ = 40.0;
  int tap_max_move_raw_ = 200;
  int tap_min_hold_ms_ = 30;
  int tap_max_hold_ms_ = 800;
  TouchCalibration touch_calib_;
  double eye_smoothing_ = 0.7;
  double eye_deadzone_ = 0.08;
  double info_refresh_hz_ = 1.0;
  double status_publish_hz_ = 1.0;
  double lifecycle_poll_hz_ = 1.0;
  double ip_refresh_sec_ = 15.0;
  std::string topic_cmd_vel_safe_;
  std::string topic_motor_feedback_;
  std::string topic_drivechain_status_;
  std::string topic_base_status_;
  std::string service_connect_;
  std::string service_drivechain_get_state_;
  std::string service_base_get_state_;
};

}  // namespace mserve_display

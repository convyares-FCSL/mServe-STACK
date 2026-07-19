#pragma once

#include <functional>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <interfaces/msg/joystick_status.hpp>
#include <interfaces/srv/set_display_mode.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace mserve_joystick {

// Plain rclcpp::Node, not lifecycle-managed — same reasoning as
// mserve_display (no hardware "connect" step of its own; the `joy` package's
// joy_node owns the actual USB device, this node only consumes /joy). See
// README.md.
class JoystickNode : public rclcpp::Node
{
public:
  explicit JoystickNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void loadParams();
  void loadJoystickMap();
  void buildButtonDispatch();

  void onJoy(sensor_msgs::msg::Joy::SharedPtr msg);
  void publishCmdVel(const sensor_msgs::msg::Joy & msg);
  void dispatchButtons(const sensor_msgs::msg::Joy & msg);

  void callConnect();
  void callDisplayInfo();
  void adjustSpeed(double delta);
  void adjustAngular(double delta);
  void publishStatus();
  void onSetOnline(
    std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);

  // Subscription / publishers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  // Whether the last publishCmdVel() call sent a nonzero Twist — see that
  // function's comment for why this gates publishing at all, not just what
  // gets published.
  bool cmd_vel_was_active_ = false;
  rclcpp::Publisher<interfaces::msg::JoystickStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_pub_timer_;

  // Service + publisher-gate
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_online_srv_;
  // Offline: /joy is still processed (button actions like connect/
  // display_info/speed+angular adjustment still work, for testing those
  // without a live robot) but /cmd_vel is force-zeroed — see publishCmdVel().
  // Defaults true (online) per its own request. Deliberately not a declared
  // ROS parameter — this is session/testing state, not deployment config.
  bool online_ = true;

  // Clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr connect_client_;
  rclcpp::Client<interfaces::srv::SetDisplayMode>::SharedPtr display_set_mode_client_;
  bool connect_in_flight_ = false;
  bool display_set_mode_in_flight_ = false;

  // Selected controller_profiles.<controller_profile_> entry (from
  // joystick_params.yaml): semantic name -> raw sensor_msgs/Joy
  // axes[]/buttons[] index (see loadJoystickMap()).
  std::map<std::string, int> axis_name_to_index_;
  std::map<std::string, int> button_name_to_index_;

  // button_actions: semantic name -> action, loaded from mserve_params.yaml.
  std::map<std::string, std::string> button_actions_;

  // Combined at startup from the two maps above: raw button index -> handler.
  std::map<int, std::function<void()>> button_index_to_handler_;
  std::map<int, int> prev_button_state_;

  // Params (loaded in loadParams())
  //
  // Deliberately no invert_linear/invert_angular here: this node publishes
  // the standard REP-103 Twist convention (stick up = +linear.x, stick left
  // = +angular.z) with no per-source sign tweaking, same as every other
  // /cmd_vel publisher (Nav2, teleop). Any actual hardware-specific
  // direction correction belongs downstream, in the one place that already
  // owns it — mserve_drivechain's hardware.motor_signs (see
  // mserve_params.yaml) — not duplicated/guessed at in each input layer.
  std::string joy_topic_;
  std::string controller_profile_;
  std::string linear_axis_name_;
  std::string angular_axis_name_;
  double deadzone_ = 0.1;
  double speed_scale_ = 0.4;
  double speed_min_ = 0.1;
  double speed_max_ = 0.8;
  double speed_step_ = 0.1;
  double angular_scale_ = 0.6;
  double angular_min_ = 0.1;
  double angular_max_ = 1.2;
  double angular_step_ = 0.2;
  double status_publish_hz_ = 2.0;
  std::string topic_cmd_vel_;
  std::string service_connect_;
  std::string service_display_set_mode_;
};

}  // namespace mserve_joystick

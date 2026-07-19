#include "mserve_joystick/joystick_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace mserve_joystick {

JoystickNode::JoystickNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mserve_joystick", options)
{
  loadParams();
  loadJoystickMap();
  buildButtonDispatch();

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(topic_cmd_vel_, rclcpp::QoS(10));
  status_pub_ = create_publisher<interfaces::msg::JoystickStatus>("~/status", rclcpp::QoS(10));
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    joy_topic_, rclcpp::QoS(10),
    std::bind(&JoystickNode::onJoy, this, std::placeholders::_1));

  connect_client_ = create_client<std_srvs::srv::Trigger>(service_connect_);
  display_set_mode_client_ =
    create_client<interfaces::srv::SetDisplayMode>(service_display_set_mode_);

  set_online_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/set_online",
    std::bind(&JoystickNode::onSetOnline, this, std::placeholders::_1, std::placeholders::_2));

  if (status_publish_hz_ > 0.0) {
    const auto period = std::chrono::duration<double>(1.0 / status_publish_hz_);
    status_pub_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&JoystickNode::publishStatus, this));
  }

  RCLCPP_INFO(
    get_logger(), "mserve_joystick ready: joy_topic='%s' -> cmd_vel='%s', speed=%.2f m/s, angular=%.2f rad/s",
    joy_topic_.c_str(), topic_cmd_vel_.c_str(), speed_scale_, angular_scale_);
}

// Resolves button_actions_ (name -> action) against button_name_to_index_
// (name -> raw buttons[] index) into a direct index -> handler dispatch, so
// onJoy()'s per-message work is a flat map lookup instead of two.
void JoystickNode::buildButtonDispatch()
{
  const std::map<std::string, std::function<void()>> action_handlers = {
    {"connect", [this] {callConnect();}},
    {"display_info", [this] {callDisplayInfo();}},
    {"increase_speed", [this] {adjustSpeed(speed_step_);}},
    {"decrease_speed", [this] {adjustSpeed(-speed_step_);}},
    {"increase_angular", [this] {adjustAngular(angular_step_);}},
    {"decrease_angular", [this] {adjustAngular(-angular_step_);}},
  };

  for (const auto & [name, action] : button_actions_) {
    const auto index_it = button_name_to_index_.find(name);
    if (index_it == button_name_to_index_.end()) {
      RCLCPP_WARN(
        get_logger(), "button_actions: '%s' has no joystick_map.buttons entry — ignored",
        name.c_str());
      continue;
    }
    const auto handler_it = action_handlers.find(action);
    if (handler_it == action_handlers.end()) {
      RCLCPP_WARN(
        get_logger(), "button_actions: unknown action '%s' for button '%s' — ignored",
        action.c_str(), name.c_str());
      continue;
    }
    button_index_to_handler_[index_it->second] = handler_it->second;
  }
}

void JoystickNode::onJoy(sensor_msgs::msg::Joy::SharedPtr msg)
{
  publishCmdVel(*msg);
  dispatchButtons(*msg);
}

void JoystickNode::publishCmdVel(const sensor_msgs::msg::Joy & msg)
{
  auto axis_value = [&msg](const std::map<std::string, int> & name_to_index,
                            const std::string & name) -> double {
      const auto it = name_to_index.find(name);
      if (it == name_to_index.end() || it->second < 0 ||
        static_cast<size_t>(it->second) >= msg.axes.size())
      {
        return 0.0;
      }
      return static_cast<double>(msg.axes[it->second]);
    };

  // No sign inversion here — see joystick_node.hpp's "Params" comment for
  // why that belongs in mserve_drivechain's motor_signs, not here.
  double linear = axis_value(axis_name_to_index_, linear_axis_name_);
  if (std::abs(linear) < deadzone_) {linear = 0.0;}

  double angular = axis_value(axis_name_to_index_, angular_axis_name_);
  if (std::abs(angular) < deadzone_) {angular = 0.0;}

  geometry_msgs::msg::Twist twist;
  // Offline: force-zeroed rather than skipping the publish entirely, so any
  // command already in flight when going offline is overwritten immediately
  // instead of waiting on mserve_base's cmd_vel_timeout_ms to zero it.
  if (online_) {
    twist.linear.x = linear * speed_scale_;
    twist.angular.z = angular * angular_scale_;
  }

  // /cmd_vel has no source arbitration yet (mserve_base is the documented
  // future home for that — see CLAUDE.md) — anything publishing here
  // competes directly with every other publisher on the same topic (e.g.
  // the web UI's drive buttons). joy_node's autorepeat (mserve_min.launch.py)
  // keeps /joy arriving continuously even with the stick centered, on
  // purpose — a genuinely-held nonzero stick still needs a steady /cmd_vel
  // stream so mserve_base's cmd_vel_timeout_ms doesn't zero it between real
  // axis changes. But blindly republishing on every single tick regardless
  // of value meant this node kept spamming zero-Twist at ~20Hz forever
  // after the very first touch of the stick, even once released back to
  // center — confirmed on real hardware: driving from the web UI afterward
  // went from smooth to "temperamental", because these idle zero commands
  // kept interleaving with and overwriting the web UI's own (slower, 10Hz)
  // publishes. Fix: publish while genuinely active (nonzero), plus exactly
  // one final zero on the active->idle transition (so the robot still
  // stops immediately, not after waiting on the timeout) — then go silent,
  // leaving /cmd_vel free for whoever else is driving.
  const bool is_active = (twist.linear.x != 0.0) || (twist.angular.z != 0.0);
  if (is_active || cmd_vel_was_active_) {
    cmd_vel_pub_->publish(twist);
  }
  cmd_vel_was_active_ = is_active;
}

void JoystickNode::dispatchButtons(const sensor_msgs::msg::Joy & msg)
{
  for (const auto & [index, handler] : button_index_to_handler_) {
    if (index < 0 || static_cast<size_t>(index) >= msg.buttons.size()) {
      continue;
    }
    const int current = msg.buttons[index];
    const int previous = prev_button_state_.count(index) ? prev_button_state_[index] : 0;
    if (previous == 0 && current == 1) {
      handler();
    }
    prev_button_state_[index] = current;
  }
}

void JoystickNode::callConnect()
{
  if (connect_in_flight_) {
    return;
  }
  if (!connect_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "connect: service '%s' unavailable", service_connect_.c_str());
    return;
  }
  connect_in_flight_ = true;
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  connect_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      connect_in_flight_ = false;
      const auto resp = future.get();
      RCLCPP_INFO(
        get_logger(), "connect: success=%d message='%s'", resp->success, resp->message.c_str());
    });
}

void JoystickNode::callDisplayInfo()
{
  if (display_set_mode_in_flight_) {
    return;
  }
  if (!display_set_mode_client_->service_is_ready()) {
    RCLCPP_WARN(
      get_logger(), "display_info: service '%s' unavailable", service_display_set_mode_.c_str());
    return;
  }
  display_set_mode_in_flight_ = true;
  auto request = std::make_shared<interfaces::srv::SetDisplayMode::Request>();
  request->mode = "info";
  display_set_mode_client_->async_send_request(
    request,
    [this](rclcpp::Client<interfaces::srv::SetDisplayMode>::SharedFuture future) {
      display_set_mode_in_flight_ = false;
      const auto resp = future.get();
      RCLCPP_INFO(
        get_logger(), "display_info: success=%d message='%s'", resp->success,
        resp->message.c_str());
    });
}

void JoystickNode::adjustSpeed(double delta)
{
  speed_scale_ = std::clamp(speed_scale_ + delta, speed_min_, speed_max_);
  RCLCPP_INFO(get_logger(), "speed scale -> %.2f m/s", speed_scale_);
}

void JoystickNode::adjustAngular(double delta)
{
  angular_scale_ = std::clamp(angular_scale_ + delta, angular_min_, angular_max_);
  RCLCPP_INFO(get_logger(), "angular scale -> %.2f rad/s", angular_scale_);
}

void JoystickNode::publishStatus()
{
  interfaces::msg::JoystickStatus msg;
  msg.controller_profile = controller_profile_;
  msg.speed_scale = speed_scale_;
  msg.angular_scale = angular_scale_;
  msg.online = online_;
  status_pub_->publish(msg);
}

void JoystickNode::onSetOnline(
  std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  online_ = request->data;
  RCLCPP_INFO(get_logger(), "online -> %s", online_ ? "true" : "false");
  response->success = true;
  response->message = online_ ? "online" : "offline (cmd_vel force-zeroed)";
}

}  // namespace mserve_joystick

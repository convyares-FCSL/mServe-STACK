#include "mserve_display/display_node.hpp"

#include "mserve_display/display_limits.hpp"

#include <algorithm>
#include <chrono>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netdb.h>
#include <sys/socket.h>

using namespace std::chrono_literals;

namespace mserve_display {

DisplayNode::DisplayNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mserve_display", options)
{
  declareParams();
  loadParams();

  framebuffer_ = std::make_unique<Framebuffer>(fb_device_);
  framebuffer_available_ = framebuffer_->open();
  if (!framebuffer_available_) {
    RCLCPP_ERROR(get_logger(), "failed to open %s — running without a display", fb_device_.c_str());
  }

  touch_ = std::make_unique<TouchInput>(
    kScreenWidth, kScreenHeight, touch_calib_, tap_max_move_raw_, tap_min_hold_ms_,
    tap_max_hold_ms_);
  touch_available_ = touch_->open(touch_device_name_match_);
  if (!touch_available_) {
    RCLCPP_ERROR(get_logger(), "touch input unavailable — display-only mode");
  }

  menu_button_rects_ = menuButtonRects(kScreenWidth, kScreenHeight);
  state_.ip_address = getIpAddress();

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    topic_cmd_vel_safe_, rclcpp::QoS(10),
    std::bind(&DisplayNode::onCmdVel, this, std::placeholders::_1));
  motor_feedback_sub_ = create_subscription<interfaces::msg::DriveMotorFeedback>(
    topic_motor_feedback_, rclcpp::QoS(10),
    std::bind(&DisplayNode::onMotorFeedback, this, std::placeholders::_1));
  drivechain_status_sub_ = create_subscription<interfaces::msg::DriveStatus>(
    topic_drivechain_status_, rclcpp::QoS(10),
    std::bind(&DisplayNode::onDrivechainStatus, this, std::placeholders::_1));
  base_status_sub_ = create_subscription<interfaces::msg::DriveStatus>(
    topic_base_status_, rclcpp::QoS(10),
    std::bind(&DisplayNode::onBaseStatus, this, std::placeholders::_1));

  connect_client_ = create_client<std_srvs::srv::Trigger>(service_connect_);
  drivechain_get_state_client_ =
    create_client<lifecycle_msgs::srv::GetState>(service_drivechain_get_state_);
  base_get_state_client_ = create_client<lifecycle_msgs::srv::GetState>(service_base_get_state_);

  set_mode_srv_ = create_service<interfaces::srv::SetDisplayMode>(
    "~/set_display_mode",
    std::bind(&DisplayNode::onSetDisplayMode, this, std::placeholders::_1, std::placeholders::_2));
  status_pub_ = create_publisher<interfaces::msg::DisplayStatus>("~/status", rclcpp::QoS(10));

  if (touch_available_ && touch_poll_hz_ > 0.0) {
    touch_poll_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / touch_poll_hz_),
      std::bind(&DisplayNode::onTouchPollTimer, this));
  }
  if (info_refresh_hz_ > 0.0) {
    info_refresh_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / info_refresh_hz_),
      std::bind(&DisplayNode::onInfoRefreshTimer, this));
  }
  if (lifecycle_poll_hz_ > 0.0) {
    lifecycle_poll_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / lifecycle_poll_hz_),
      std::bind(&DisplayNode::onLifecyclePollTimer, this));
  }
  if (status_publish_hz_ > 0.0) {
    status_pub_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / status_publish_hz_),
      std::bind(&DisplayNode::onStatusPublishTimer, this));
  }
  if (ip_refresh_sec_ > 0.0) {
    ip_refresh_timer_ = create_wall_timer(
      std::chrono::duration<double>(ip_refresh_sec_),
      std::bind(&DisplayNode::onIpRefreshTimer, this));
  }

  requestRedraw();
  RCLCPP_INFO(get_logger(), "mserve_display up — screen=face, fb=%s", framebuffer_available_ ? "ok" : "unavailable");
}

void DisplayNode::onCmdVel(geometry_msgs::msg::Twist::SharedPtr msg)
{
  double target = eye_max_angular_ > 0.0 ? msg->angular.z / eye_max_angular_ : 0.0;
  target = std::clamp(target, -1.0, 1.0);
  state_.eye_direction = state_.eye_direction * eye_smoothing_ + target * (1.0 - eye_smoothing_);

  if (current_screen_ == Screen::Face &&
    std::abs(state_.eye_direction - last_rendered_eye_direction_) > eye_deadzone_)
  {
    last_rendered_eye_direction_ = state_.eye_direction;
    requestRedraw();
  }
}

void DisplayNode::onMotorFeedback(interfaces::msg::DriveMotorFeedback::SharedPtr msg)
{
  state_.motor_feedback = *msg;
}

void DisplayNode::onDrivechainStatus(interfaces::msg::DriveStatus::SharedPtr msg)
{
  state_.drivechain_status = *msg;
}

void DisplayNode::onBaseStatus(interfaces::msg::DriveStatus::SharedPtr msg)
{
  state_.base_status = *msg;
}

void DisplayNode::onTouchPollTimer()
{
  if (!touch_available_) {
    return;
  }
  auto tap = touch_->poll();
  if (tap) {
    RCLCPP_INFO(
      get_logger(), "tap: raw=(%d,%d) screen=(%d,%d) — use for touch_calib.* tuning",
      tap->raw_x, tap->raw_y, tap->screen_x, tap->screen_y);
    onTap(*tap);
  }
}

void DisplayNode::onTap(const TapEvent & tap)
{
  switch (current_screen_) {
    case Screen::Face:
      setScreen(Screen::Menu);
      break;
    case Screen::Menu: {
        MenuButton btn = hitTestMenu(tap.screen_x, tap.screen_y, menu_button_rects_);
        if (btn == MenuButton::Connect) {
          callConnect();
        } else if (btn == MenuButton::Info) {
          setScreen(Screen::Info);
        } else if (btn == MenuButton::Face) {
          setScreen(Screen::Face);
        }
        break;
      }
    case Screen::Info:
      setScreen(Screen::Menu);
      break;
  }
}

void DisplayNode::onInfoRefreshTimer()
{
  if (current_screen_ == Screen::Info) {
    requestRedraw();
  }
}

void DisplayNode::pollLifecycleState(
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client, std::string * state_out)
{
  if (!client->service_is_ready()) {
    *state_out = "unavailable";
    return;
  }
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  // state_out is a pointer to a long-lived DisplayNode member (state_.*),
  // captured by value (the pointer, not a reference) — safe to use once
  // this callback fires later, unlike a reference to the reference
  // parameter above, which would already be gone by then.
  client->async_send_request(
    request,
    [state_out](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
      *state_out = future.get()->current_state.label;
    });
}

void DisplayNode::onLifecyclePollTimer()
{
  pollLifecycleState(drivechain_get_state_client_, &state_.drivechain_state);
  pollLifecycleState(base_get_state_client_, &state_.base_state);
}

void DisplayNode::onStatusPublishTimer()
{
  publishStatus();
}

void DisplayNode::onIpRefreshTimer()
{
  state_.ip_address = getIpAddress();
}

void DisplayNode::onSetDisplayMode(
  interfaces::srv::SetDisplayMode::Request::SharedPtr request,
  interfaces::srv::SetDisplayMode::Response::SharedPtr response)
{
  auto screen = screenFromName(request->mode);
  if (!screen) {
    response->success = false;
    response->message = "unknown mode: " + request->mode;
    return;
  }
  setScreen(*screen);
  response->success = true;
  response->message = "ok";
}

void DisplayNode::callConnect()
{
  if (state_.connect_in_flight) {
    return;
  }
  if (!connect_client_->service_is_ready()) {
    state_.connect_result_valid = true;
    state_.connect_result_success = false;
    state_.connect_result_message = "service unavailable";
    requestRedraw();
    return;
  }

  state_.connect_in_flight = true;
  state_.connect_result_valid = false;
  requestRedraw();

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  // Async, non-blocking — spin_until_future_complete here would freeze
  // touch polling/redraws on this node's single-threaded executor while
  // waiting on the drivechain's UART handshake (up to a few seconds).
  connect_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto resp = future.get();
      state_.connect_in_flight = false;
      state_.connect_result_valid = true;
      state_.connect_result_success = resp->success;
      state_.connect_result_message = resp->message;
      if (current_screen_ == Screen::Menu) {
        requestRedraw();
      }
    });
}

void DisplayNode::setScreen(Screen s)
{
  current_screen_ = s;
  state_.connect_result_valid = false;  // don't carry a stale result across navigation
  requestRedraw();
  publishStatus();
}

void DisplayNode::requestRedraw()
{
  if (!framebuffer_available_) {
    return;
  }
  switch (current_screen_) {
    case Screen::Face:
      renderFace(*framebuffer_, state_);
      break;
    case Screen::Menu:
      renderMenu(*framebuffer_, state_, menu_button_rects_);
      break;
    case Screen::Info:
      renderInfo(*framebuffer_, state_);
      break;
  }
  framebuffer_->present();
}

void DisplayNode::publishStatus()
{
  interfaces::msg::DisplayStatus msg;
  msg.mode = screenName(current_screen_);
  if (state_.connect_in_flight) {
    msg.text = "connecting...";
  } else if (state_.connect_result_valid) {
    msg.text = state_.connect_result_success ? "connected" : state_.connect_result_message;
  } else {
    msg.text = "";
  }
  status_pub_->publish(msg);
}

std::string DisplayNode::screenName(Screen s)
{
  switch (s) {
    case Screen::Face: return "face";
    case Screen::Menu: return "menu";
    case Screen::Info: return "info";
  }
  return "face";
}

std::optional<Screen> DisplayNode::screenFromName(const std::string & name)
{
  std::string lower;
  lower.resize(name.size());
  std::transform(name.begin(), name.end(), lower.begin(), ::tolower);
  if (lower == "face") {return Screen::Face;}
  if (lower == "menu") {return Screen::Menu;}
  if (lower == "info") {return Screen::Info;}
  return std::nullopt;
}

std::string DisplayNode::getIpAddress() const
{
  ifaddrs * ifaddr = nullptr;
  if (getifaddrs(&ifaddr) == -1) {
    return "unknown";
  }

  std::string result = "none";
  for (ifaddrs * ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) {
      continue;
    }
    if (std::string(ifa->ifa_name) == "lo") {
      continue;
    }
    if (!(ifa->ifa_flags & IFF_UP)) {
      continue;
    }
    char host[NI_MAXHOST];
    int rc = getnameinfo(
      ifa->ifa_addr, sizeof(sockaddr_in), host, NI_MAXHOST, nullptr, 0, NI_NUMERICHOST);
    if (rc == 0) {
      result = host;
      break;
    }
  }
  freeifaddrs(ifaddr);
  return result;
}

}  // namespace mserve_display

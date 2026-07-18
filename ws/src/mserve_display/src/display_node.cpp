#include "mserve_display/display_node.hpp"

#include "mserve_display/display_limits.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netdb.h>
#include <sys/socket.h>

using namespace std::chrono_literals;

namespace mserve_display {

namespace
{
int64_t nowMs()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}
}  // namespace

DisplayNode::DisplayNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mserve_display", options)
{
  declareParams();
  loadParams();

  framebuffer_ = std::make_unique<Framebuffer>(fb_device_, fb_flip_180_);
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
  // Polls at a fraction of the timeout, not once — the timeout only needs
  // to fire close to kMenuInfoTimeoutMs after the last tap, not exactly on
  // it.
  idle_timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(kMenuInfoTimeoutMs / 5),
    std::bind(&DisplayNode::onIdleTimeoutTimer, this));

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
  // Any tap counts as activity, even a Menu tap that misses every button
  // (MenuButton::None) and so never reaches setScreen() below.
  last_screen_activity_ms_ = nowMs();

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
        } else if (btn == MenuButton::Calibrate) {
          setScreen(Screen::Calibrate);
        }
        break;
      }
    case Screen::Info:
      setScreen(Screen::Menu);
      break;
    case Screen::Calibrate:
      onCalibrateTap(tap);
      break;
  }
}

void DisplayNode::onCalibrateTap(const TapEvent & tap)
{
  int step = state_.calib_step;
  if (step < 0 || step > 3) {
    return;
  }

  int64_t now = nowMs();
  if (step > 0 && now - last_calib_tap_ms_ < kCalibTapDebounceMs) {
    RCLCPP_WARN(
      get_logger(), "calibrate: ignored tap %ldms after the last one (debounce=%dms) — "
      "an accidental double-tap on the same prompt", static_cast<long>(now - last_calib_tap_ms_),
      kCalibTapDebounceMs);
    return;
  }
  last_calib_tap_ms_ = now;

  state_.calib_raw_x[step] = tap.raw_x;
  state_.calib_raw_y[step] = tap.raw_y;
  state_.calib_step = step + 1;

  if (state_.calib_step >= 4) {
    applyCalibration();
    setScreen(Screen::Menu);
    return;
  }
  requestRedraw();
}

void DisplayNode::applyCalibration()
{
  // Step order: 0=Up, 1=Down, 2=Left, 3=Right (see onTap's Menu case /
  // DisplayState::calib_step comment). Which raw axis (ABS_X or ABS_Y)
  // actually corresponds to screen X isn't assumed — it's derived from
  // whichever axis varies more between the Left and Right taps, so this
  // self-corrects for both swap_xy and either axis being inverted, not
  // just a fixed-range assumption.
  int up_x = state_.calib_raw_x[0], up_y = state_.calib_raw_y[0];
  int down_x = state_.calib_raw_x[1], down_y = state_.calib_raw_y[1];
  int left_x = state_.calib_raw_x[2], left_y = state_.calib_raw_y[2];
  int right_x = state_.calib_raw_x[3], right_y = state_.calib_raw_y[3];

  int dx_horiz = right_x - left_x;
  int dy_horiz = right_y - left_y;
  bool swap_xy = std::abs(dy_horiz) > std::abs(dx_horiz);

  int screenx_left = swap_xy ? left_y : left_x;
  int screenx_right = swap_xy ? right_y : right_x;
  int screeny_up = swap_xy ? up_x : up_y;
  int screeny_down = swap_xy ? down_x : down_y;

  TouchCalibration calib;
  calib.x_min = std::min(screenx_left, screenx_right);
  calib.x_max = std::max(screenx_left, screenx_right);
  calib.invert_x = screenx_left > screenx_right;
  calib.y_min = std::min(screeny_up, screeny_down);
  calib.y_max = std::max(screeny_up, screeny_down);
  calib.invert_y = screeny_up > screeny_down;
  calib.swap_xy = swap_xy;

  touch_calib_ = calib;
  if (touch_) {
    touch_->setCalibration(calib);
  }
  RCLCPP_INFO(
    get_logger(),
    "touch calibrated: x=[%d,%d] invert_x=%d y=[%d,%d] invert_y=%d swap_xy=%d — "
    "update touch_calib.* in mserve_params.yaml to persist this across restarts",
    calib.x_min, calib.x_max, calib.invert_x, calib.y_min, calib.y_max, calib.invert_y,
    calib.swap_xy);
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
  last_screen_activity_ms_ = nowMs();  // Menu/Info auto-return clock — see kMenuInfoTimeoutMs
  state_.connect_result_valid = false;  // don't carry a stale result across navigation
  if (s == Screen::Calibrate) {
    state_.calib_step = 0;  // always start the wizard fresh, even entered via ~/set_display_mode
  }
  requestRedraw();
  publishStatus();
}

void DisplayNode::onIdleTimeoutTimer()
{
  bool timeoutable = current_screen_ == Screen::Menu || current_screen_ == Screen::Info;
  if (timeoutable && nowMs() - last_screen_activity_ms_ >= kMenuInfoTimeoutMs) {
    setScreen(Screen::Face);
  }
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
    case Screen::Calibrate:
      renderCalibrate(*framebuffer_, state_);
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
    case Screen::Calibrate: return "calibrate";
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
  if (lower == "calibrate") {
    return Screen::Calibrate;
  }
  return std::nullopt;
}

std::string DisplayNode::getIpAddress() const
{
  // In Docker mode this process runs inside the container's own network
  // namespace — getifaddrs() below would only ever see the bridge-network
  // IP (172.20.x.x, per docker-compose.yml's default compose network), not
  // the host's real LAN IP, no matter which interface is picked. Confirmed
  // on real hardware: Info screen showed 172.20.0.2 while the actual
  // reachable address was the host's 172.16.68.73. run_stack.sh exports
  // MSERVE_HOST_IP (computed on the host, outside the container) and
  // docker-compose.yml passes it through — prefer that when set. Native
  // (non-Docker) mode never sets this, so it falls through to the
  // getifaddrs() scan below unchanged, where it already works correctly.
  const char * host_ip_env = std::getenv("MSERVE_HOST_IP");
  if (host_ip_env != nullptr && host_ip_env[0] != '\0') {
    return host_ip_env;
  }

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

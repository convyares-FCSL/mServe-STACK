#include "include/base_bt_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace mserve_base {

// bb_get: read a value from the blackboard; return def if the key is absent.
template <typename T>
static T bb_get(const BT::Blackboard::Ptr & bb, const std::string & key, const T & def = T{})
{
  T v = def;
  (void)bb->get(key, v);
  return v;
}

static rclcpp::Logger get_ros_logger(const BT::NodeConfig & cfg)
{
  rclcpp::Logger logger = rclcpp::get_logger("base_bt");
  (void)cfg.blackboard->get("ros_logger", logger);
  return logger;
}

// ==============================================================================
// Drive tree actions
// ==============================================================================

ApplyCmdVelSafety::ApplyCmdVelSafety(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::NodeStatus ApplyCmdVelSafety::tick()
{
  auto & bb = config().blackboard;

  CmdVelStore * store = nullptr;
  (void)bb->get("cmd_vel_store", store);

  const int    timeout_ms  = bb_get(bb, std::string("cmd_vel_timeout_ms"), 500);
  const double max_linear  = bb_get(bb, std::string("max_linear_speed"),  0.8);
  const double max_angular = bb_get(bb, std::string("max_angular_speed"), 1.2);

  geometry_msgs::msg::Twist safe{};
  if (store && store->age_ms() < static_cast<double>(timeout_ms)) {
    const auto raw = store->latest();
    safe.linear.x  = std::clamp(raw.linear.x,  -max_linear,  max_linear);
    safe.angular.z = std::clamp(raw.angular.z, -max_angular, max_angular);
  }
  // else: stale or no /cmd_vel yet — leave safe at zero (dead-man switch)

  bb->set("safe_twist", safe);

  using PublishFn = std::function<void(const geometry_msgs::msg::Twist &)>;
  PublishFn pub_fn;
  if (bb->get("publish_cmd_vel_safe", pub_fn)) pub_fn(safe);

  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

ComputeKinematics::ComputeKinematics(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::NodeStatus ComputeKinematics::tick()
{
  auto & bb = config().blackboard;

  geometry_msgs::msg::Twist safe{};
  (void)bb->get("safe_twist", safe);

  const double wheel_separation = bb_get(bb, std::string("wheel_separation"), 0.35);
  const double wheel_radius     = bb_get(bb, std::string("wheel_radius"),     0.08);
  const double gear_ratio       = bb_get(bb, std::string("gear_ratio"),       1.0);
  const int    left_id          = bb_get(bb, std::string("left_motor_id"),  2);
  const int    right_id         = bb_get(bb, std::string("right_motor_id"), 1);

  const double v = safe.linear.x;
  const double w = safe.angular.z;

  // Differential-drive kinematics: Twist -> per-wheel angular velocity -> motor RPM
  const double left_wheel_rad_s  = (v - w * wheel_separation / 2.0) / wheel_radius;
  const double right_wheel_rad_s = (v + w * wheel_separation / 2.0) / wheel_radius;

  constexpr double kRadPerSecToRpm = 60.0 / (2.0 * M_PI);
  auto to_rpm = [&](double wheel_rad_s) -> int16_t {
    const double motor_rpm = wheel_rad_s * kRadPerSecToRpm * gear_ratio;
    return static_cast<int16_t>(std::clamp(motor_rpm,
      static_cast<double>(kMotorRpmMin), static_cast<double>(kMotorRpmMax)));
  };

  std::vector<interfaces::msg::MotorCommand> wheel_commands(2);
  wheel_commands[0].motor_id = static_cast<uint8_t>(left_id);
  wheel_commands[0].rpm      = to_rpm(left_wheel_rad_s);
  wheel_commands[1].motor_id = static_cast<uint8_t>(right_id);
  wheel_commands[1].rpm      = to_rpm(right_wheel_rad_s);

  bb->set("wheel_commands", wheel_commands);
  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

CallDriveService::CallDriveService(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::NodeStatus CallDriveService::tick()
{
  auto & bb = config().blackboard;

  rclcpp::Client<interfaces::srv::Drive>::SharedPtr client;
  if (!bb->get("drive_client", client) || !client || !client->service_is_ready()) {
    bb->set("drivechain_reachable", false);
    return BT::NodeStatus::SUCCESS;  // best-effort — drivechain not up yet
  }
  bb->set("drivechain_reachable", true);

  std::vector<interfaces::msg::MotorCommand> wheel_commands;
  (void)bb->get("wheel_commands", wheel_commands);

  auto request = std::make_shared<interfaces::srv::Drive::Request>();
  request->motor_commands = wheel_commands;

  auto logger = get_ros_logger(config());
  client->async_send_request(request,
    [logger](rclcpp::Client<interfaces::srv::Drive>::SharedFuture future) {
      auto resp = future.get();
      if (!resp->success) {
        RCLCPP_WARN(logger, "drivechain rejected drive command: %s", resp->message.c_str());
      }
    });

  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

PublishBaseStatus::PublishBaseStatus(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::NodeStatus PublishBaseStatus::tick()
{
  using DriveStatus = interfaces::msg::DriveStatus;
  using PublishFn   = std::function<void(const DriveStatus &)>;

  auto & bb = config().blackboard;
  PublishFn pub_fn;
  if (!bb->get("publish_base_status", pub_fn)) return BT::NodeStatus::SUCCESS;

  const bool reachable = bb_get(bb, std::string("drivechain_reachable"), false);

  DriveStatus msg;
  msg.status        = reachable ? "bridging" : "drivechain_unreachable";
  msg.battery_level = 0.0f;
  msg.board_alive   = reachable;
  pub_fn(msg);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mserve_base

#include "include/base_bt_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "mserve_base/base_limits.hpp"

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

// -------------------------------------------------------------------------------
// Odometry
// -------------------------------------------------------------------------------

namespace {

// Wrap an angle into (-pi, pi].
double wrap_delta(double angle) {
  while (angle > M_PI)  angle -= 2.0 * M_PI;
  while (angle <= -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// Find a motor's feedback entry by ID; nullptr if not present in this message.
const interfaces::msg::MotorState * find_motor(
  const std::vector<interfaces::msg::MotorState> & motors, int motor_id)
{
  for (const auto & m : motors) {
    if (m.motor_id == motor_id) return &m;
  }
  return nullptr;
}

}  // namespace

UpdateOdometry::UpdateOdometry(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::NodeStatus UpdateOdometry::tick()
{
  auto & bb = config().blackboard;

  interfaces::msg::DriveMotorFeedback feedback;
  if (!bb->get("motor_feedback", feedback)) return BT::NodeStatus::SUCCESS;  // no feedback yet

  const int left_id  = bb_get(bb, std::string("left_motor_id"),  2);
  const int right_id = bb_get(bb, std::string("right_motor_id"), 1);

  const auto * left  = find_motor(feedback.motors, left_id);
  const auto * right = find_motor(feedback.motors, right_id);
  if (!left || !right) return BT::NodeStatus::SUCCESS;  // feedback doesn't cover both wheels yet

  const double wheel_radius     = bb_get(bb, std::string("wheel_radius"),     0.08);
  const double wheel_separation = bb_get(bb, std::string("wheel_separation"), 0.35);
  const double gear_ratio       = bb_get(bb, std::string("gear_ratio"),       1.0);

  // mserve_drivechain's DDSM115 protocol only reports position feedback in
  // position-control mode (mode 3) — in the speed-loop mode mserve_base
  // actually drives in, position_rad is always 0 (drivechain_uart.cpp's
  // parse_json_feedback: "not returned in speed-loop feedback"). So odometry
  // integrates from wheel *velocity* instead — the one feedback signal
  // that's reliable in speed-loop mode — rather than differencing an
  // absolute position. mserve_base owns the wheel joint angle accumulation
  // the same way, for the same reason (see PublishOdometry).
  const double left_wheel_rad_s  = static_cast<double>(left->velocity_rads)  / gear_ratio;
  const double right_wheel_rad_s = static_cast<double>(right->velocity_rads) / gear_ratio;

  // DriveMotorFeedback.stamp isn't populated by mserve_drivechain either
  // (always zero) — use wall-clock time for dt instead.
  const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
  rclcpp::Time prev_time = now;
  (void)bb->get("prev_odom_time", prev_time);
  bb->set("prev_odom_time", now);

  const bool initialized = bb_get(bb, std::string("odom_initialized"), false);
  if (!initialized) {
    // First feedback since (re)configure — no dt to integrate over yet.
    bb->set("odom_initialized",  true);
    bb->set("linear_velocity",   0.0);
    bb->set("angular_velocity",  0.0);
    return BT::NodeStatus::SUCCESS;
  }

  const double dt = (now - prev_time).seconds();
  if (dt <= 0.0) return BT::NodeStatus::SUCCESS;  // duplicate/out-of-order feedback — skip

  const double left_dist  = left_wheel_rad_s  * wheel_radius * dt;
  const double right_dist = right_wheel_rad_s * wheel_radius * dt;

  const double delta_s     = (left_dist + right_dist) / 2.0;
  const double delta_theta = (right_dist - left_dist) / wheel_separation;

  double x = 0.0, y = 0.0, theta = 0.0;
  double left_wheel_angle = 0.0, right_wheel_angle = 0.0;
  (void)bb->get("odom_x", x);
  (void)bb->get("odom_y", y);
  (void)bb->get("odom_theta", theta);
  (void)bb->get("left_wheel_angle",  left_wheel_angle);
  (void)bb->get("right_wheel_angle", right_wheel_angle);

  // Midpoint (2nd-order) integration — more accurate than a first-order
  // Euler step for the same tick rate, and no more expensive to compute.
  const double theta_mid = theta + delta_theta / 2.0;
  x += delta_s * std::cos(theta_mid);
  y += delta_s * std::sin(theta_mid);
  theta = wrap_delta(theta + delta_theta);  // sum, then normalize into (-pi, pi]

  // Self-integrated wheel rotation for /joint_states — see the comment above
  // on why this can't just come from mserve_drivechain's position_rad.
  // Left unwrapped (continuously accumulating): a "continuous" URDF joint
  // doesn't require bounding, and unwrapped values preserve turn count.
  left_wheel_angle  += left_wheel_rad_s  * dt;
  right_wheel_angle += right_wheel_rad_s * dt;

  bb->set("odom_x", x);
  bb->set("odom_y", y);
  bb->set("odom_theta", theta);
  bb->set("left_wheel_angle",  left_wheel_angle);
  bb->set("right_wheel_angle", right_wheel_angle);
  bb->set("linear_velocity",  delta_s / dt);
  bb->set("angular_velocity", delta_theta / dt);

  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

PublishOdometry::PublishOdometry(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::NodeStatus PublishOdometry::tick()
{
  auto & bb = config().blackboard;

  double x = 0.0, y = 0.0, theta = 0.0, v = 0.0, w = 0.0;
  (void)bb->get("odom_x", x);
  (void)bb->get("odom_y", y);
  (void)bb->get("odom_theta", theta);
  (void)bb->get("linear_velocity",  v);
  (void)bb->get("angular_velocity", w);

  const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();

  // Pure-yaw quaternion — no roll/pitch on a ground-plane diff-drive robot.
  const double half = theta / 2.0;
  const double qz = std::sin(half);
  const double qw = std::cos(half);

  using PublishOdomFn = std::function<void(const nav_msgs::msg::Odometry &)>;
  PublishOdomFn publish_odom;
  if (bb->get("publish_odom", publish_odom)) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";
    odom.pose.pose.position.x    = x;
    odom.pose.pose.position.y    = y;
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;
    odom.twist.twist.linear.x  = v;
    odom.twist.twist.angular.z = w;
    // Covariance left at zero (default) — no real uncertainty estimate yet;
    // fine for RViz/basic use, but tune before feeding this into Nav2/AMCL.
    publish_odom(odom);
  }

  using PublishTfFn = std::function<void(const geometry_msgs::msg::TransformStamped &)>;
  PublishTfFn publish_tf;
  if (bb->get("publish_odom_tf", publish_tf)) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = now;
    tf.header.frame_id = "odom";
    tf.child_frame_id  = "base_link";
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.rotation.z = qz;
    tf.transform.rotation.w = qw;
    publish_tf(tf);
  }

  using PublishJointsFn = std::function<void(const sensor_msgs::msg::JointState &)>;
  PublishJointsFn publish_joints;
  if (bb->get("publish_joint_states", publish_joints)) {
    // Position comes from UpdateOdometry's self-integrated angle, not
    // mserve_drivechain's position_rad (always 0 in speed-loop mode — see
    // UpdateOdometry's comment). Velocity can still come straight from the
    // latest feedback, which reports speed reliably.
    double left_angle = 0.0, right_angle = 0.0;
    (void)bb->get("left_wheel_angle",  left_angle);
    (void)bb->get("right_wheel_angle", right_angle);

    double left_vel = 0.0, right_vel = 0.0;
    interfaces::msg::DriveMotorFeedback feedback;
    if (bb->get("motor_feedback", feedback)) {
      const int left_id  = bb_get(bb, std::string("left_motor_id"),  2);
      const int right_id = bb_get(bb, std::string("right_motor_id"), 1);
      const double gear_ratio = bb_get(bb, std::string("gear_ratio"), 1.0);
      const auto * left  = find_motor(feedback.motors, left_id);
      const auto * right = find_motor(feedback.motors, right_id);
      if (left)  left_vel  = static_cast<double>(left->velocity_rads)  / gear_ratio;
      if (right) right_vel = static_cast<double>(right->velocity_rads) / gear_ratio;
    }

    sensor_msgs::msg::JointState joints;
    joints.header.stamp = now;
    joints.name     = {"left_wheel_joint", "right_wheel_joint"};
    joints.position = {left_angle, right_angle};
    joints.velocity = {left_vel, right_vel};
    publish_joints(joints);
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mserve_base

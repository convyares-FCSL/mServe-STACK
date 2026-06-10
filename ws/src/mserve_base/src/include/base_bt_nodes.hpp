#pragma once

// ==============================================================================
// Base BT nodes
//
// Classification rule:
//   Instantaneous gate, no side-effects  → BT::ConditionNode
//   One-shot compute / publish op        → BT::SyncActionNode
//   Multi-step / retry op                → BT::StatefulActionNode
//
// Shared objects on the blackboard:
//   "cmd_vel_store"   CmdVelStore*
//   "drive_client"    rclcpp::Client<interfaces::srv::Drive>::SharedPtr
//
// Config written by load_params:
//   "max_linear_speed"   double — limits.max_linear_speed  (m/s)
//   "max_angular_speed"  double — limits.max_angular_speed (rad/s)
//   "wheel_separation"   double — geometry.wheel_separation (m)
//   "wheel_radius"       double — geometry.wheel_radius     (m)
//   "gear_ratio"         double — geometry.gear_ratio
//   "left_motor_id"      int    — motor_ids.left  (mserve_drivechain motor ID)
//   "right_motor_id"     int    — motor_ids.right (mserve_drivechain motor ID)
//   "cmd_vel_timeout_ms" int    — zero the output if no /cmd_vel received within this window
//   "feedback_rate"      double — drive loop rate (Hz)
//
// Runtime state passed between drive_tree nodes:
//   "safe_twist"          geometry_msgs::msg::Twist                — clamped, dead-man-gated
//   "wheel_commands"      std::vector<interfaces::msg::MotorCommand> — [left, right]
//   "drivechain_reachable" bool — set by CallDriveService, read by PublishBaseStatus
//
// Publisher functions on the blackboard (set in on_configure):
//   "publish_cmd_vel_safe" std::function<void(const geometry_msgs::msg::Twist&)>
//   "publish_base_status"  std::function<void(const DriveStatus&)>
// ==============================================================================

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <interfaces/msg/drive_status.hpp>
#include <interfaces/msg/motor_command.hpp>
#include <interfaces/srv/drive.hpp>
#include <rclcpp/rclcpp.hpp>

#include "base_types.hpp"

namespace mserve_base {

// --- Drive tree actions -------------------------------------------------------

// Read cmd_vel_store; if the cached /cmd_vel is older than cmd_vel_timeout_ms,
// substitute a zero Twist (dead-man switch). Clamp linear.x/angular.z to
// max_linear_speed/max_angular_speed. Write "safe_twist" to the blackboard and
// publish it via publish_cmd_vel_safe. Always SUCCESS.
class ApplyCmdVelSafety : public BT::SyncActionNode {
public:
  ApplyCmdVelSafety(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// Read "safe_twist" + geometry params, run differential-drive kinematics, and
// write "wheel_commands" ([left, right] interfaces::msg::MotorCommand) to the
// blackboard. Always SUCCESS.
class ComputeKinematics : public BT::SyncActionNode {
public:
  ComputeKinematics(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// Send "wheel_commands" to mserve_drivechain's ~/drive service (async,
// fire-and-forget). Sets "drivechain_reachable" on the blackboard. Best-effort:
// always SUCCESS, even if the service is unavailable.
class CallDriveService : public BT::SyncActionNode {
public:
  CallDriveService(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// Read "drivechain_reachable" from the blackboard and call publish_base_status.
class PublishBaseStatus : public BT::SyncActionNode {
public:
  PublishBaseStatus(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

}  // namespace mserve_base

#pragma once

// ==============================================================================
// Drivechain BT nodes
//
// Classification rule:
//   Instantaneous gate, no side-effects  → BT::ConditionNode
//   One-shot hardware / publish op       → BT::SyncActionNode
//   Multi-step / retry op                → BT::StatefulActionNode
//
// All nodes read shared objects from the blackboard:
//   "uart"              DriveUart*
//   "cmd_vel_cache"     CmdVelCache*
//
// Config scalars on the blackboard (written by load_params):
//   "left_motor_id"      int
//   "right_motor_id"     int
//   "max_rpm"            int
//   "cmd_vel_timeout_ms" int
//   "wheel_separation"   double  (m)
//   "wheel_radius"       double  (m)
//
// Runtime state on the blackboard (written by drive loop nodes):
//   "uart_connected"    bool
//   "left_rpm"          int     ← written by ComputeRpm
//   "right_rpm"         int     ← written by ComputeRpm
//   "left_speed_fb"     double  ← written by SetMotorSpeed (rad/s)
//   "right_speed_fb"    double  ← written by SetMotorSpeed (rad/s)
//   "left_pos_fb"       double  ← written by SetMotorSpeed (rad)
//   "right_pos_fb"      double  ← written by SetMotorSpeed (rad)
//   "left_fault"        int     ← written by SetMotorSpeed
//   "right_fault"       int     ← written by SetMotorSpeed
//
// Publisher functions on the blackboard (set in on_configure):
//   "publish_wheel_feedback"   std::function<void(const WheelFeedback&)>
//   "publish_drive_status"     std::function<void(const DriveStatus&)>
// ==============================================================================

#include <behaviortree_cpp/bt_factory.h>
#include <interfaces/msg/drive_status.hpp>
#include <interfaces/msg/wheel_feedback.hpp>

#include "drivechain_uart.hpp"
#include "drivechain_cmd_vel_cache.hpp"

namespace mserve_drivechain {

// --- Conditions ---------------------------------------------------------------

// SUCCESS while uart->is_open() (gates the drive sequence).
class UartOpen : public BT::ConditionNode {
public:
  UartOpen(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// SUCCESS if last feedback fault_code == 0 for motor_id.
class MotorHealthy : public BT::ConditionNode {
public:
  MotorHealthy(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// --- Connect / stop tree actions ----------------------------------------------

// Open UART at uart_device / baud from blackboard. Sets uart_connected on success.
class OpenUart : public BT::SyncActionNode {
public:
  OpenUart(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// Close UART and clear uart_connected.
class CloseUart : public BT::SyncActionNode {
public:
  CloseUart(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// Send stop to motor_id and verify echo — confirms motor is alive.
class PingMotor : public BT::SyncActionNode {
public:
  PingMotor(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// Send 0xA0 mode packet. DDSM115: mode=2 for closed-loop speed.
class SetMotorMode : public BT::SyncActionNode {
public:
  SetMotorMode(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// Zero a motor (best-effort, always SUCCESS).
class StopMotor : public BT::SyncActionNode {
public:
  StopMotor(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// Send change-id (5× 0xAA 0x55 0x53) then ping new_id to verify.
class SetMotorId : public BT::StatefulActionNode {
public:
  SetMotorId(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart()   override;
  BT::NodeStatus onRunning() override;
  void           onHalted()  override {}
private:
  bool result_ = false;
};

// --- Drive tree actions -------------------------------------------------------

// Read cmd_vel cache → apply diff-drive + max_rpm clamp → write left_rpm / right_rpm.
// If cache is stale (age > cmd_vel_timeout_ms), writes 0/0 instead.
// Always SUCCESS.
class ComputeRpm : public BT::SyncActionNode {
public:
  ComputeRpm(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// Command a motor at {rpm}, read back feedback, write speed/pos/fault to blackboard.
// motor_id determines whether "left_*" or "right_*" keys are written.
class SetMotorSpeed : public BT::SyncActionNode {
public:
  SetMotorSpeed(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// Read left/right speed + position from blackboard, call publish_wheel_feedback fn.
class PublishWheelFeedback : public BT::SyncActionNode {
public:
  PublishWheelFeedback(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// Read uart_connected + sim_mode from blackboard, call publish_drive_status fn.
class PublishDriveStatus : public BT::SyncActionNode {
public:
  PublishDriveStatus(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

}  // namespace mserve_drivechain

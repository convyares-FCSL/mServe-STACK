#pragma once

// ==============================================================================
// Drivechain BT nodes
//
// Classification rule:
//   Instantaneous gate, no side-effects  → BT::ConditionNode
//   One-shot hardware / publish op       → BT::SyncActionNode
//   Multi-step / retry op                → BT::StatefulActionNode
//
// Shared objects on the blackboard:
//   "uart"            DriveUart*
//   "drive_cmd_store" DriveCommandStore*
//
// Config written by load_params:
//   "motor_list"         std::vector<MotorDescriptor>
//   "command_timeout_ms" int    — zero all motors if no command received within this window
//   "feedback_rate"      double — drive loop rate (Hz)
//
// Runtime state written by drive loop nodes:
//   "uart_connected"     bool
//   "motor_states"       std::vector<interfaces::msg::MotorState>  ← written by SetAllMotors
//
// Publisher functions on the blackboard (set in on_configure):
//   "publish_motor_feedback"   std::function<void(const DriveMotorFeedback&)>
//   "publish_drive_status"     std::function<void(const DriveStatus&)>
// ==============================================================================

#include <unordered_map>

#include <behaviortree_cpp/bt_factory.h>
#include <interfaces/msg/drive_motor_feedback.hpp>
#include <interfaces/msg/drive_status.hpp>

#include "drivechain_types.hpp"
#include "drivechain_uart.hpp"

namespace mserve_drivechain {

// --- Conditions ---------------------------------------------------------------

// SUCCESS while uart->is_open() (gates the drive sequence).
class UartOpen : public BT::ConditionNode {
public:
  UartOpen(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// SUCCESS if motor_id has fault_code == 0 in motor_states.
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

// Ping a single motor by ID (used by set_id_tree).
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

// Zero a single motor (used by set_id_tree; best-effort, always SUCCESS).
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

// --- N-motor connect / stop ---------------------------------------------------

// Ping + set_mode(2) for every enabled motor in motor_list (3 ping attempts each).
class ConnectAllMotors : public BT::SyncActionNode {
public:
  ConnectAllMotors(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// Zero every enabled motor in motor_list (best-effort, always SUCCESS).
class StopAllMotors : public BT::SyncActionNode {
public:
  StopAllMotors(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// --- Drive tree actions -------------------------------------------------------

// Read motor_commands cache → apply per-motor sign → set_speed on each enabled
// motor → write motor_states to blackboard.
// If cache is stale (age > command_timeout_ms), sends 0 RPM to all motors.
// Always SUCCESS.
class SetAllMotors : public BT::SyncActionNode {
public:
  SetAllMotors(const std::string & name, const BT::NodeConfig & cfg);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// Read motor_states from blackboard and call publish_motor_feedback fn.
class PublishMotorFeedback : public BT::SyncActionNode {
public:
  PublishMotorFeedback(const std::string & name, const BT::NodeConfig & cfg);
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

#pragma once

// ==============================================================================
// Subsystem BT nodes
//
// Classification rule (enforced by BTcpp):
//   Instantaneous gate (SUCCESS/FAILURE only) → BT::ConditionNode
//   Wait until state reached (may return RUNNING) → BT::StatefulActionNode
//   Hardware command (async service/action call) → RosServiceNode / RosActionNode
//
// Telemetry pattern:
//   If this subsystem reads telemetry, own the subscription in SubsystemNode and
//   share a thread-safe cache via the blackboard (see hyfleet_booster for the
//   BoosterTelemetryCache pattern). BT nodes never hold subscriptions directly.
//
// TODO: declare your BT node classes here.
//
// Examples:
//
//   // Hardware command — async service call
//   class StartDevice : public BT::RosServiceNode<mserve_interfaces::srv::SubsystemCmd> {
//   public:
//       StartDevice(const std::string& name, const BT::NodeConfig& config, BT::RosNodeParams params);
//       static BT::PortsList providedPorts();
//       bool setRequest(Request::SharedPtr & req) override;
//       BT::NodeStatus onResponseReceived(const Response::SharedPtr & res) override;
//       BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
//   };
//
//   // Wait node — blocks until condition met
//   class DeviceAtState : public BT::StatefulActionNode {
//   public:
//       DeviceAtState(const std::string& name, const BT::NodeConfig& config);
//       static BT::PortsList providedPorts();
//       BT::NodeStatus onStart()   override;
//       BT::NodeStatus onRunning() override;
//       void           onHalted()  override;
//   };
//
//   // Gate node — instantaneous check
//   class DeviceSafe : public BT::ConditionNode {
//   public:
//       DeviceSafe(const std::string& name, const BT::NodeConfig& config);
//       static BT::PortsList providedPorts();
//       BT::NodeStatus tick() override;
//   };
// ==============================================================================

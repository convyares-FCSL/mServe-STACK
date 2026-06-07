#pragma once

#include <string>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <mserve_interfaces/action/control_booster.hpp>
#include "mserve_interfaces/srv/compressor_cmd.hpp"

namespace hyfleet_compressor {

using ControlBooster = mserve_interfaces::action::ControlBooster;

// ==============================================================================
// BoostCmdBase — shared RosActionNode implementation for both boosters.
// BoostLow and BoostHigh differ only in which blackboard keys they write
// feedback to — pressure_key_ and percent_key_ are set by each subclass.
// ==============================================================================

class BoostCmdBase : public BT::RosActionNode<ControlBooster> {
public:
    // Ports: server_name (which action server to call) plus the four goal fields.
    // server_name is read from the blackboard at tree tick time, so the same node
    // class can call either booster depending on what the coordinator puts there.
    static BT::PortsList providedPorts();

    // Fill the outgoing ControlBooster goal from blackboard keys.
    bool setGoal(Goal& goal) override;

    // Receive mid-flight feedback from the booster and write to blackboard.
    // tick_tree_once() in CompressorNode reads these keys each tick and publishes
    // them on the active ControlCompressor goal handle (the coordinator's client).
    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

    // Final result — succeed or fail the BT node.
    BT::NodeStatus onResultReceived(const WrappedResult& result) override;

    // ROS layer error — server unreachable, timeout, etc.
    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

protected:
    BoostCmdBase(
        const std::string & name,
        const BT::NodeConfig & config,
        const BT::RosNodeParams & params,
        std::string pressure_key,
        std::string percent_key);

private:
    std::string pressure_key_;
    std::string percent_key_;
};

// ==============================================================================
// BoostLow — calls /low_booster/control_booster.
// Writes feedback to "low_pressure" and "low_percent_complete".
// ==============================================================================

class BoostLow : public BoostCmdBase {
public:
    BoostLow(
        const std::string & name,
        const BT::NodeConfig & config,
        const BT::RosNodeParams & params)
    : BoostCmdBase(name, config, params, "low_pressure", "low_percent_complete") {}
};

// ==============================================================================
// BoostHigh — calls /high_booster/control_booster.
// Writes feedback to "high_pressure" and "high_percent_complete".
// ==============================================================================

class BoostHigh : public BoostCmdBase {
public:
    BoostHigh(
        const std::string & name,
        const BT::NodeConfig & config,
        const BT::RosNodeParams & params)
    : BoostCmdBase(name, config, params, "high_pressure", "high_percent_complete") {}
};


// ==============================================================================
// BT action nodes :  ControlSV
// ==============================================================================
 
class ControlSV : public BT::RosServiceNode<mserve_interfaces::srv::CompressorCmd> {
    public:
        ControlSV(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params);

        // Define the input ports for the node
        static BT::PortsList providedPorts() ;

        // Construct the service request based on the input ports and send it.
        bool setRequest(Request::SharedPtr& request) override ;

        // Process the service response and return the appropriate BT status.
        BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override ;

    private:

};

}  // namespace hyfleet_compressor

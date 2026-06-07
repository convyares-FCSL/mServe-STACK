#pragma once

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <mserve_interfaces/srv/booster_cmd.hpp>

namespace hyfleet_booster {

// ==============================================================================
// BT action nodes :  StartVFD (Cmd = 1)
// ==============================================================================

class StartVFD : public BT::RosServiceNode<mserve_interfaces::srv::BoosterCmd> {
    public:
        StartVFD(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params);

        // Define the input ports for the node
        static BT::PortsList providedPorts() ;

        // Construct the service request based on the input ports and send it.
        bool setRequest(Request::SharedPtr& request) override ;

        // Process the service response and return the appropriate BT status.
        BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override ;

    private:

};

// ==============================================================================
// BT action nodes :  StopVFD (Cmd = 2)
// ==============================================================================

class StopVFD : public BT::RosServiceNode<mserve_interfaces::srv::BoosterCmd> {
    public:
        StopVFD(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params);

        // Define the input ports for the node
        static BT::PortsList providedPorts() ;

        // Construct the service request based on the input ports and send it.
        bool setRequest(Request::SharedPtr& request) override ;

        // Process the service response and return the appropriate BT status.
        BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override ;

    private:

};

// ==============================================================================
// SetPCSV — plain service call: enable/disable PCSV, SUCCESS when PLC confirms.
// ==============================================================================

class SetPCSV : public BT::RosServiceNode<mserve_interfaces::srv::BoosterCmd> {
public:
    SetPCSV(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);
    static BT::PortsList providedPorts();
    bool           setRequest(Request::SharedPtr& request) override;
    BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
};

// ==============================================================================
// HoldPCSV — state owner: stays RUNNING while PCSV is on, fires PCSV-off on halt.
// Positioned after SetPCSV(enable=true) in a Sequence inside a Parallel so that
// any exit path (InletGuard pause, target reached, goal abort) drops PCSV.
// ==============================================================================

class HoldPCSV : public BT::StatefulActionNode {
public:
    HoldPCSV(const std::string& name, const BT::NodeConfig& config,
             std::shared_ptr<rclcpp::Node> bt_node);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
private:
    using BoosterCmd = mserve_interfaces::srv::BoosterCmd;
    std::shared_ptr<rclcpp::Node>         bt_node_;
    rclcpp::Client<BoosterCmd>::SharedPtr client_;
};

// ==============================================================================
// BT action nodes :  ControlSV (Cmd = 4)
// ==============================================================================

class ControlSV : public BT::RosServiceNode<mserve_interfaces::srv::BoosterCmd> {
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

} // namespace hyfleet_booster

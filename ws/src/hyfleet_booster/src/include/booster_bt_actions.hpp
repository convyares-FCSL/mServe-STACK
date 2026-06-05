#pragma once

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_ros2/bt_service_node.hpp>
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
// BT action nodes :  SetPCSV (Cmd = 3)
// ==============================================================================
 
class SetPCSV : public BT::RosServiceNode<mserve_interfaces::srv::BoosterCmd> {
    public:
        SetPCSV(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params);

        // Define the input ports for the node
        static BT::PortsList providedPorts() ;

        // Construct the service request based on the input ports and send it.
        bool setRequest(Request::SharedPtr& request) override ;

        // Process the service response and return the appropriate BT status.
        BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override ;

    private:

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


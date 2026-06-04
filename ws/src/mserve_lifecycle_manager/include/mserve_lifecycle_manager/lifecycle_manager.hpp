#pragma once

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/bt_service_node.hpp>

namespace lifecyclemanager {

// ==============================================================================
// BT checker nodes for lifecycle state management
// ==============================================================================
 
class IsInState : public BT::RosServiceNode<lifecycle_msgs::srv::GetState> {
    public:
        IsInState(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params);

        // Define the input ports for the node
        static BT::PortsList providedPorts() ;

        // Construct the service request based on the input ports and send it.
        bool setRequest(Request::SharedPtr& request) override;

        // Process the service response and return the appropriate BT status.
        BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override ;

    private:
        std::string node_name_;
};

// ==============================================================================
// BT action nodes for lifecycle state management
// ==============================================================================
 
class ChangeStateNode : public BT::RosServiceNode<lifecycle_msgs::srv::ChangeState> {
    public:
        ChangeStateNode(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params);

        // Define the input ports for the node
        static BT::PortsList providedPorts() ;

        // Construct the service request based on the input ports and send it.
        bool setRequest(Request::SharedPtr& request) override ;

        // Process the service response and return the appropriate BT status.
        BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override ;

    private:
        std::string node_name_;
};

// ==============================================================================
// ROS 2 Lifecycle Manager class
// ==============================================================================
 
class LifecycleManager : public rclcpp::Node {
    public:
        LifecycleManager() : Node("lifecycle_manager") {};

        void build() ;

    private:

    BT::Tree tree_;
    rclcpp::TimerBase::SharedPtr tick_timer_;
    
};

} // namespace lifecyclemanager
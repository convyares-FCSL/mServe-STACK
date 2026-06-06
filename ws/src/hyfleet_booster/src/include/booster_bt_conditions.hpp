#pragma once

#include <deque>
#include "rclcpp/rclcpp.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <mserve_interfaces/msg/compressor_telemetry.hpp>


namespace hyfleet_booster {

// ==============================================================================
// BT condition nodes :  Inlet Pressure Stable
// ==============================================================================
 
class InletPressureStable : public BT::RosTopicSubNode<mserve_interfaces::msg::CompressorTelemetry> {
    public:
        InletPressureStable(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params);

        // Define the ports for the node
        static BT::PortsList providedPorts() ;

        // Receives the most recent message and returns SUCCESS or FAILURE
        BT::NodeStatus onTick(const std::shared_ptr<mserve_interfaces::msg::CompressorTelemetry>& last_msg) override;

        // Keep last message
        inline bool latchLastMessage() const override { return true; }

    private:
        std::deque<double> window_;
        builtin_interfaces::msg::Time last_stamp_{};
};


// ==============================================================================
// BT condition nodes :  VFD At Speed
// ==============================================================================
 
class VFDAtSpeed : public BT::RosTopicSubNode<mserve_interfaces::msg::CompressorTelemetry> {
    public:
        VFDAtSpeed(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params);

        // Define the ports for the node
        static BT::PortsList providedPorts() ;

        // Receives the most recent message and returns SUCCESS or FAILURE
        BT::NodeStatus onTick(const std::shared_ptr<mserve_interfaces::msg::CompressorTelemetry>& last_msg) override;

        // Keep last message
        inline bool latchLastMessage() const override { return true; }

    private:

};

// ==============================================================================
// BT condition nodes :  VFD Stopped
// ==============================================================================
 
class VFDStopped : public BT::RosTopicSubNode<mserve_interfaces::msg::CompressorTelemetry> {
    public:
        VFDStopped(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params);

        // Define the ports for the node
        static BT::PortsList providedPorts() ;

        // Receives the most recent message and returns SUCCESS or FAILURE
        BT::NodeStatus onTick(const std::shared_ptr<mserve_interfaces::msg::CompressorTelemetry>& last_msg) override;

        // Keep last message
        inline bool latchLastMessage() const override { return true; }

    private:

};

// ==============================================================================
// BT condition nodes :  Outlet At Pressure 
// ==============================================================================
 
class OutletAtPressure : public BT::RosTopicSubNode<mserve_interfaces::msg::CompressorTelemetry> {
    public:
        OutletAtPressure(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params);

        // Define the ports for the node
        static BT::PortsList providedPorts() ;

        // Receives the most recent message and returns SUCCESS or FAILURE
        BT::NodeStatus onTick(const std::shared_ptr<mserve_interfaces::msg::CompressorTelemetry>& last_msg) override;

        // Keep last message
        inline bool latchLastMessage() const override { return true; }

    private:

};

// ==============================================================================
// BT condition nodes :  Inlet Pressure Safe 
// ==============================================================================
 
class InletPressureSafe : public BT::RosTopicSubNode<mserve_interfaces::msg::CompressorTelemetry> {
    public:
        InletPressureSafe(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params);

        // Define the ports for the node
        static BT::PortsList providedPorts() ;

        // Receives the most recent message and returns SUCCESS or FAILURE
        BT::NodeStatus onTick(const std::shared_ptr<mserve_interfaces::msg::CompressorTelemetry>& last_msg) override;

        // Keep last message
        inline bool latchLastMessage() const override { return true; }
        
    private:

};

} // namespace hyfleet_booster


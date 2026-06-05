#include "include/booster_bt_conditions.hpp"

namespace hyfleet_booster {

// ==============================================================================
// BT condition nodes :  InletPressureStable
// ==============================================================================
    
 InletPressureStable::InletPressureStable(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params)
: BT::RosTopicSubNode<mserve_interfaces::msg::CompressorTelemetry>(name, config, params) {}

BT::PortsList InletPressureStable::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<int>("inlet_pt_index")
    });
}

BT::NodeStatus InletPressureStable::onTick(const std::shared_ptr<mserve_interfaces::msg::CompressorTelemetry>& last_msg) {
    (void)last_msg;
    return BT::NodeStatus::FAILURE; // TODO: rolling window stability check
}


// ==============================================================================
// BT condition nodes :  VFD At Speed
// ==============================================================================

 VFDAtSpeed::VFDAtSpeed(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params)
: BT::RosTopicSubNode<mserve_interfaces::msg::CompressorTelemetry>(name, config, params) {}

BT::PortsList VFDAtSpeed::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<int>("vfd_index"),
        BT::InputPort<double>("target_speed"),
    });
}

BT::NodeStatus VFDAtSpeed::onTick(const std::shared_ptr<mserve_interfaces::msg::CompressorTelemetry>& last_msg) {
    if (!last_msg) {
        RCLCPP_WARN(logger(), "VFDAtSpeed: no telemetry received yet");
        return BT::NodeStatus::FAILURE;
    }

    auto index_res = getInput<int>("vfd_index");
    auto target_res = getInput<double>("target_speed");

    if (!index_res || !target_res) {
        RCLCPP_ERROR(logger(), "VFDAtSpeed: missing input ports");
        return BT::NodeStatus::FAILURE;
    }

    const double speed = last_msg->vfd_speed_rpm[index_res.value()];
    const double target = target_res.value();

    // TODO switch to a band target +- a SP
    return speed >= target ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}


// ==============================================================================
// BT condition nodes :  Outlet At Pressure 
// ==============================================================================
 
OutletAtPressure::OutletAtPressure(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params)
: BT::RosTopicSubNode<mserve_interfaces::msg::CompressorTelemetry>(name, config, params) {}

BT::PortsList OutletAtPressure::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<int>("outlet_pt_index"),
        BT::InputPort<double>("target_pressure"),
    });
}

BT::NodeStatus OutletAtPressure::onTick(const std::shared_ptr<mserve_interfaces::msg::CompressorTelemetry>& last_msg) {
    if (!last_msg) {
        RCLCPP_WARN(logger(), "OutletAtPressure: no telemetry received yet");
        return BT::NodeStatus::FAILURE;
    }

    auto index_res = getInput<int>("outlet_pt_index");
    auto target_res = getInput<double>("target_pressure");

    if (!index_res || !target_res) {
        RCLCPP_ERROR(logger(), "OutletAtPressure: missing input ports");
        return BT::NodeStatus::FAILURE;
    }

    const double pressure = last_msg->hbu_pt_bar[index_res.value()];
    const double target = target_res.value();

    return pressure >= target ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ==============================================================================
// BT condition nodes :  Inlet Pressure Safe 
// ==============================================================================
 
InletPressureSafe::InletPressureSafe(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params)
: BT::RosTopicSubNode<mserve_interfaces::msg::CompressorTelemetry>(name, config, params) {}

BT::PortsList InletPressureSafe::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<int>("inlet_pt_index"),
        BT::InputPort<double>("safe_pressure"),
    });
}

BT::NodeStatus InletPressureSafe::onTick(const std::shared_ptr<mserve_interfaces::msg::CompressorTelemetry>& last_msg) {
    if (!last_msg) {
        RCLCPP_WARN(logger(), "InletPressureSafe: no telemetry received yet");
        return BT::NodeStatus::FAILURE;
    }

    auto index_res = getInput<int>("inlet_pt_index");
    auto target_res = getInput<double>("safe_pressure");

    if (!index_res || !target_res) {
        RCLCPP_ERROR(logger(), "InletPressureSafe: missing input ports");
        return BT::NodeStatus::FAILURE;
    }

    const double pressure = last_msg->hbu_pt_bar[index_res.value()];
    const double target = target_res.value();

    return pressure >= target ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace hyfleet_booster
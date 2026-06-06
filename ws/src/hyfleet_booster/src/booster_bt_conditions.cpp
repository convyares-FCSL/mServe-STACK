#include "include/booster_bt_conditions.hpp"
#include <algorithm>

namespace hyfleet_booster {

// ==============================================================================
// BT condition nodes :  InletPressureStable
// ==============================================================================
    
 InletPressureStable::InletPressureStable(const std::string& name, const BT::NodeConfiguration& config,  const BT::RosNodeParams& params)
: BT::RosTopicSubNode<mserve_interfaces::msg::CompressorTelemetry>(name, config, params) {}

BT::PortsList InletPressureStable::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<int>("inlet_pt_index"),
        BT::InputPort<int>("stabilization_samples"),
        BT::InputPort<double>("stability_tolerance")
    });
}

BT::NodeStatus InletPressureStable::onTick(const std::shared_ptr<mserve_interfaces::msg::CompressorTelemetry>& last_msg) {
    if (!last_msg) {
        RCLCPP_WARN(logger(), "InletPressureStable: no telemetry received yet");
        return BT::NodeStatus::FAILURE;
    }

    auto index_res = getInput<int>("inlet_pt_index");
    auto samples_res = getInput<int>("stabilization_samples");
    auto tolerance_res = getInput<double>("stability_tolerance");

    if (!index_res || !samples_res || !tolerance_res) {
        RCLCPP_ERROR(logger(), "InletPressureStable: missing input ports");
        return BT::NodeStatus::FAILURE;
    }

    const int inlet_pt_index = index_res.value();
    const int stabilization_samples = samples_res.value();
    const double stability_tolerance = tolerance_res.value();

    if (stabilization_samples <= 0) {
        RCLCPP_ERROR(logger(), "InletPressureStable: stabilization_samples must be > 0");
        return BT::NodeStatus::FAILURE;
    }

    if (last_msg->timestamp != last_stamp_) {
        const double pressure = last_msg->pt_bar[inlet_pt_index];

        window_.push_back(pressure);

        while (window_.size() > static_cast<std::size_t>(stabilization_samples)) {
            window_.pop_front();
        }

        last_stamp_ = last_msg->timestamp;
    }

    if (window_.size() < static_cast<std::size_t>(stabilization_samples)) {
        return BT::NodeStatus::RUNNING;
    }

    const auto [min_it, max_it] = std::minmax_element(window_.begin(), window_.end());
    const double variation = *max_it - *min_it;

    return variation <= stability_tolerance ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
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
// BT condition nodes :  VFD Stopped
// ==============================================================================

 VFDStopped::VFDStopped(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params)
: BT::RosTopicSubNode<mserve_interfaces::msg::CompressorTelemetry>(name, config, params) {}

BT::PortsList VFDStopped::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<int>("vfd_index"),
        BT::InputPort<double>("stop_threshold"),
    });
}

BT::NodeStatus VFDStopped::onTick(const std::shared_ptr<mserve_interfaces::msg::CompressorTelemetry>& last_msg) {
    if (!last_msg) {
        RCLCPP_WARN(logger(), "VFDStopped: no telemetry received yet");
        return BT::NodeStatus::FAILURE;
    }

    auto index_res = getInput<int>("vfd_index");
    auto threshold_res = getInput<double>("stop_threshold");

    if (!index_res || !threshold_res) {
        RCLCPP_ERROR(logger(), "VFDStopped: missing input ports");
        return BT::NodeStatus::FAILURE;
    }

    const double speed = last_msg->vfd_speed_rpm[index_res.value()];
    const uint8_t state = last_msg->vfd_state[index_res.value()];
    const double threshold = threshold_res.value();

    using Msg = mserve_interfaces::msg::CompressorTelemetry;
    if (speed > threshold || state == Msg::VFD_RUNNING) {
        RCLCPP_WARN(logger(), "VFDStopped: not stopped — speed: %.1f rpm, state: %d", speed, state);
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(logger(), "VFDStopped: stopped — speed: %.1f rpm, state: %d", speed, state);
    return BT::NodeStatus::SUCCESS;
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

    const double pressure = last_msg->pt_bar[index_res.value()];
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

    const double pressure = last_msg->pt_bar[index_res.value()];
    const double target = target_res.value();

    return pressure >= target ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace hyfleet_booster
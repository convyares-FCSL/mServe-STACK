#include "include/booster_bt_actions.hpp"

namespace hyfleet_booster {

// ==============================================================================
// BT action nodes :  StartVFD (Cmd = 1)
// ==============================================================================
    
StartVFD::StartVFD(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) : BT::RosServiceNode<mserve_interfaces::srv::BoosterCmd>(name, config, params) {
    auto it = config.input_ports.find("service_name");
    if (it == config.input_ports.end()) {
        throw std::runtime_error("StartVFD: required port 'service_name' not found");
    }
    setServiceName(it->second);
}

BT::PortsList StartVFD::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<std::string>("service_name"),
        BT::InputPort<double>("speed_rpm"),
    });
}

bool StartVFD::setRequest(Request::SharedPtr& request) {
    // Get the speed name from input port
    auto speed_res = getInput<double>("speed_rpm");
    if (!speed_res) {
        RCLCPP_ERROR(logger(), "Missing input port [speed]: %s", speed_res.error().c_str());
        return false;
    }
    const auto& speed = speed_res.value();

    // Check if the speed name is valid
    //TODO get config and check aginst speed range

    request->cmd = Request::START_VFD;
    request->speed_rpm = speed;
    return true;
}

BT::NodeStatus StartVFD::onResponseReceived(const Response::SharedPtr& response) {
    if (!response->success) {
        RCLCPP_ERROR(logger(), "VFD start request failed, msg : %s", response->message.c_str()); 
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(logger(), "VFD start request success");
    return BT::NodeStatus::SUCCESS;
    }

// ==============================================================================
// BT action nodes :  StopVFD (Cmd = 2)
// ==============================================================================
    
StopVFD::StopVFD(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) : BT::RosServiceNode<mserve_interfaces::srv::BoosterCmd>(name, config, params) {
    auto it = config.input_ports.find("service_name");
    if (it == config.input_ports.end()) {
        throw std::runtime_error("StopVFD: required port 'service_name' not found");
    }
    setServiceName(it->second);
}

BT::PortsList StopVFD::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<std::string>("service_name")
    });
}

bool StopVFD::setRequest(Request::SharedPtr& request) {
    request->cmd = Request::STOP_VFD;
    return true;
}

BT::NodeStatus StopVFD::onResponseReceived(const Response::SharedPtr& response) {
    if (!response->success) {
        RCLCPP_ERROR(logger(), "VFD stop request failed, msg : %s", response->message.c_str()); 
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(logger(), "VFD stop request success");
    return BT::NodeStatus::SUCCESS;
    }

// ==============================================================================
// BT action nodes :  SetPCSV (Cmd = 3)
// ==============================================================================
    
SetPCSV::SetPCSV(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) : BT::RosServiceNode<mserve_interfaces::srv::BoosterCmd>(name, config, params) {
    auto it = config.input_ports.find("service_name");
    if (it == config.input_ports.end()) {
        throw std::runtime_error("SetPCSV: required port 'service_name' not found");
    }
    setServiceName(it->second);
}

BT::PortsList SetPCSV::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<std::string>("service_name"),
        BT::InputPort<bool>("enable"),
        BT::InputPort<double>("cpm")
    });
}

bool SetPCSV::setRequest(Request::SharedPtr& request) {
    // Get the enable state from input port
    auto enable_res = getInput<bool>("enable");
    if (!enable_res) {
        RCLCPP_ERROR(logger(), "Missing input port [enable]: %s", enable_res.error().c_str());
        return false;
    }
    const auto& enable = enable_res.value();

    // Get the cpm state from input port
    auto cpm_res = getInput<double>("cpm");
    if (!cpm_res) {
        RCLCPP_ERROR(logger(), "Missing input port [cpm]: %s", cpm_res.error().c_str());
        return false;
    }
    const auto& cpm = cpm_res.value();
    // Check if the cpm is valid
    //TODO get config and check aginst cpm range - decide cap or reject ?? 

    request->cmd = Request::SET_PCSV;
    request->enable = enable;
    request->cpm = cpm;
    
    return true;
}

BT::NodeStatus SetPCSV::onResponseReceived(const Response::SharedPtr& response) {
    if (!response->success) {
        RCLCPP_ERROR(logger(), "Set PCSV request failed, msg : %s", response->message.c_str()); 
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(logger(), "Set PCSV request success");
    return BT::NodeStatus::SUCCESS;
}

// ==============================================================================
// BT action nodes :  ControlSV (Cmd = 4)
// ==============================================================================
    
ControlSV::ControlSV(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) : BT::RosServiceNode<mserve_interfaces::srv::BoosterCmd>(name, config, params) {
    auto it = config.input_ports.find("service_name");
    if (it == config.input_ports.end()) {
        throw std::runtime_error("ControlSV: required port 'service_name' not found");
    }
    setServiceName(it->second);
}

BT::PortsList ControlSV::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<std::string>("service_name"),
        BT::InputPort<bool>("enable"),
        BT::InputPort<std::string>("device_id")
    });
}

bool ControlSV::setRequest(Request::SharedPtr& request) {
    // Get the enable state from input port
    auto enable_res = getInput<bool>("enable");
    if (!enable_res) {
        RCLCPP_ERROR(logger(), "Missing input port [enable]: %s", enable_res.error().c_str());
        return false;
    }
    const auto& enable = enable_res.value();

    // Get the device_id from input port
    auto device_id_res = getInput<std::string>("device_id");
    if (!device_id_res) {
        RCLCPP_ERROR(logger(), "Missing input port [device_id]: %s", device_id_res.error().c_str());
        return false;
    }
    const auto& device_id= device_id_res.value();
    // Check if the device_id is valid - TODO

    request->cmd = Request::CONTROL_SV;
    request->enable = enable;
    request->device_id = device_id;
    
    return true;
}

BT::NodeStatus ControlSV::onResponseReceived(const Response::SharedPtr& response) {
    if (!response->success) {
        RCLCPP_ERROR(logger(), "Set SV request failed, msg : %s", response->message.c_str()); 
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(logger(), "Set SV request success");
    return BT::NodeStatus::SUCCESS;
}

} // namespace hyfleet_booster
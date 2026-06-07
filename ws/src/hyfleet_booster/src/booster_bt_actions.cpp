#include "include/booster_bt_actions.hpp"

namespace hyfleet_booster {

// ==============================================================================
// BT action nodes :  StartVFD (Cmd = 1)
// ==============================================================================

StartVFD::StartVFD(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) : BT::RosServiceNode<mserve_interfaces::srv::BoosterCmd>(name, config, params) {
}

BT::PortsList StartVFD::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<std::string>("service_name"),
        BT::InputPort<double>("speed_rpm"),
    });
}

bool StartVFD::setRequest(Request::SharedPtr& request) {
    auto speed_res = getInput<double>("speed_rpm");
    if (!speed_res) {
        RCLCPP_ERROR(logger(), "Missing input port [speed]: %s", speed_res.error().c_str());
        return false;
    }
    request->cmd = Request::START_VFD;
    request->setpoint = speed_res.value();
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
// SetPCSV
// ==============================================================================

SetPCSV::SetPCSV(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params)
    : BT::RosServiceNode<mserve_interfaces::srv::BoosterCmd>(name, config, params) {}

BT::PortsList SetPCSV::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<std::string>("service_name"),
        BT::InputPort<bool>("enable"),
        BT::InputPort<double>("cpm", 0.0, "Cycles per minute (only used when enable=true)"),
    });
}

bool SetPCSV::setRequest(Request::SharedPtr& request) {
    auto enable_res = getInput<bool>("enable");
    auto cpm_res    = getInput<double>("cpm");
    if (!enable_res) {
        RCLCPP_ERROR(logger(), "SetPCSV: missing enable port");
        return false;
    }
    request->cmd      = Request::SET_PCSV;
    request->enable   = enable_res.value();
    request->setpoint = cpm_res ? cpm_res.value() : 0.0;
    return true;
}

BT::NodeStatus SetPCSV::onResponseReceived(const Response::SharedPtr& response) {
    if (!response->success) {
        RCLCPP_ERROR(logger(), "SetPCSV request failed: %s", response->message.c_str());
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(logger(), "Set PCSV request success");
    return BT::NodeStatus::SUCCESS;
}

// ==============================================================================
// HoldPCSV
// ==============================================================================

HoldPCSV::HoldPCSV(const std::string& name, const BT::NodeConfig& config,
                   std::shared_ptr<rclcpp::Node> bt_node)
    : BT::StatefulActionNode(name, config), bt_node_(std::move(bt_node)) {}

BT::PortsList HoldPCSV::providedPorts() {
    return { BT::InputPort<std::string>("service_name") };
}

BT::NodeStatus HoldPCSV::onStart() {
    if (!client_) {
        auto svc = getInput<std::string>("service_name");
        if (!svc) {
            RCLCPP_ERROR(rclcpp::get_logger(name()), "HoldPCSV: missing service_name port");
            return BT::NodeStatus::FAILURE;
        }
        client_ = bt_node_->create_client<BoosterCmd>(svc.value());
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoldPCSV::onRunning() {
    return BT::NodeStatus::RUNNING;
}

void HoldPCSV::onHalted() {
    if (client_) {
        auto req      = std::make_shared<BoosterCmd::Request>();
        req->cmd      = BoosterCmd::Request::SET_PCSV;
        req->enable   = false;
        req->setpoint = 0.0;
        client_->async_send_request(req);
    }
}

// ==============================================================================
// BT action nodes :  ControlSV (Cmd = 4)
// ==============================================================================

ControlSV::ControlSV(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) : BT::RosServiceNode<mserve_interfaces::srv::BoosterCmd>(name, config, params) {
}

BT::PortsList ControlSV::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<std::string>("service_name"),
        BT::InputPort<bool>("enable"),
        BT::InputPort<uint8_t>("sv_index")
    });
}

bool ControlSV::setRequest(Request::SharedPtr& request) {
    auto enable_res = getInput<bool>("enable");
    if (!enable_res) {
        RCLCPP_ERROR(logger(), "Missing input port [enable]: %s", enable_res.error().c_str());
        return false;
    }
    auto sv_index_res = getInput<uint8_t>("sv_index");
    if (!sv_index_res) {
        RCLCPP_ERROR(logger(), "Missing input port [sv_index]: %s", sv_index_res.error().c_str());
        return false;
    }
    request->cmd = Request::CONTROL_SV;
    request->enable = enable_res.value();
    request->index = sv_index_res.value();
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

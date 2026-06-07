#include "include/compressor_bt_nodes.hpp"
#include <rclcpp_action/types.hpp>

namespace hyfleet_compressor {

// ==============================================================================
// BoostCmdBase
// ==============================================================================

BoostCmdBase::BoostCmdBase(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params,
    std::string pressure_key, std::string percent_key) : BT::RosActionNode<ControlBooster>(name, config, params)
    , pressure_key_(std::move(pressure_key)) , percent_key_(std::move(percent_key)) {}

BT::PortsList BoostCmdBase::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<uint8_t>("command"),
        BT::InputPort<double>("target_pressure"),
        BT::InputPort<double>("cpm"),
        BT::InputPort<double>("speed_rpm"),
});
}

bool BoostCmdBase::setGoal(Goal & goal)
{
    auto cmd      = getInput<uint8_t>("command");
    auto pressure = getInput<double>("target_pressure");
    auto cpm      = getInput<double>("cpm");
    auto speed    = getInput<double>("speed_rpm");

    if (!cmd)      { RCLCPP_ERROR(logger(), "BoostCmd: missing port [command]: %s",         cmd.error().c_str());      return false; }
    if (!pressure) { RCLCPP_ERROR(logger(), "BoostCmd: missing port [target_pressure]: %s", pressure.error().c_str()); return false; }
    if (!cpm)      { RCLCPP_ERROR(logger(), "BoostCmd: missing port [cpm]: %s",             cpm.error().c_str());      return false; }
    if (!speed)    { RCLCPP_ERROR(logger(), "BoostCmd: missing port [speed_rpm]: %s",       speed.error().c_str());    return false; }

    goal.command         = cmd.value();
    goal.target_pressure = pressure.value();
    goal.cpm             = cpm.value();
    goal.speed_rpm       = speed.value();
    return true;
}

BT::NodeStatus BoostCmdBase::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
    // Write mid-flight booster telemetry into the blackboard.
    // CompressorNode reads these each tick and re-publishes on its own goal handle.
    config().blackboard->set(pressure_key_, feedback->pressure);
    config().blackboard->set(percent_key_,  feedback->percent_complete);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BoostCmdBase::onResultReceived(const WrappedResult & result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED
        && result.result
        && result.result->accepted)
    {
        RCLCPP_INFO(logger(), "BoostCmd: goal succeeded");
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(logger(), "BoostCmd: goal failed — code=%d msg=%s",
        static_cast<int>(result.code),
        result.result ? result.result->message.c_str() : "no result");
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus BoostCmdBase::onFailure(BT::ActionNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "BoostCmd: action error — %s", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
}

// ==============================================================================
// BT action nodes :  ControlSV
// ==============================================================================
    
ControlSV::ControlSV(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) : BT::RosServiceNode<mserve_interfaces::srv::CompressorCmd>(name, config, params) {
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

}  // namespace hyfleet_compressor

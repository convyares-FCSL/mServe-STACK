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
        BT::InputPort<uint8_t>("on_target",        uint8_t{0},  "0=SUCCEED 1=HOLD on target reached"),
        BT::InputPort<uint8_t>("on_inlet_starve",  uint8_t{0},  "0=ABORT 1=PAUSE on inlet starvation"),
        BT::InputPort<double>("inlet_starve_bar",  -1.0, "Inlet starve threshold bar (-1=use booster param)"),
        BT::InputPort<double>("inlet_resume_bar",  -1.0, "Inlet resume threshold bar (-1=use booster param)"),
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

    auto on_target   = getInput<uint8_t>("on_target");
    auto starve_mode = getInput<uint8_t>("on_inlet_starve");
    auto starve_bar  = getInput<double>("inlet_starve_bar");
    auto resume_bar  = getInput<double>("inlet_resume_bar");
    goal.on_target        = on_target   ? on_target.value()   : uint8_t{0};
    goal.on_inlet_starve  = starve_mode ? starve_mode.value() : uint8_t{0};
    goal.inlet_starve_bar = starve_bar  ? starve_bar.value()  : -1.0;
    goal.inlet_resume_bar = resume_bar  ? resume_bar.value()  : -1.0;
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

// ==============================================================================
// InterstageAboveBand
// ==============================================================================

BT::NodeStatus InterstageAboveBand::onRunning() {
    std::shared_ptr<CompressorTelemetryCache> cache;
    if (!config().blackboard->get("telemetry_cache", cache) || !cache) {
        RCLCPP_ERROR(rclcpp::get_logger("InterstageAboveBand"), "telemetry_cache not on blackboard");
        return BT::NodeStatus::FAILURE;
    }
    int pt_idx = 7;
    double threshold = 200.0;
    (void)config().blackboard->get("interstage_pt_index", pt_idx);
    (void)config().blackboard->get("interstage_start_threshold_bar", threshold);
    auto [msg, stamp] = cache->latest();
    if (!msg) return BT::NodeStatus::RUNNING;
    const double pressure = msg->pt_bar[pt_idx];
    if (pressure >= threshold) {
        RCLCPP_INFO(rclcpp::get_logger("InterstageAboveBand"),
            "Interstage %.1f bar >= %.1f bar — opening interstage SV", pressure, threshold);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

}  // namespace hyfleet_compressor

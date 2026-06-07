#include "include/booster_bt_conditions.hpp"
#include "include/booster_telemetry_cache.hpp"
#include <algorithm>
#include <cmath>

namespace hyfleet_booster {

// ==============================================================================
// WAIT — Inlet Pressure Stable
// ==============================================================================

InletPressureStable::InletPressureStable(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config) {}

BT::PortsList InletPressureStable::providedPorts() {
    return {
        BT::InputPort<int>("inlet_pt_index"),
        BT::InputPort<int>("stabilization_samples"),
        BT::InputPort<double>("stability_tolerance"),
        BT::InputPort<int>("stability_timeout_ms")
    };
}

BT::NodeStatus InletPressureStable::onStart() {
    auto timeout_res = getInput<int>("stability_timeout_ms");
    if (!timeout_res) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "InletPressureStable: missing stability_timeout_ms port");
        return BT::NodeStatus::FAILURE;
    }
    timeout_    = std::chrono::milliseconds(timeout_res.value());
    start_time_ = std::chrono::steady_clock::now();
    window_.clear();
    last_stamp_ = builtin_interfaces::msg::Time();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus InletPressureStable::onRunning() {
    if (std::chrono::steady_clock::now() - start_time_ > timeout_) {
        RCLCPP_WARN(rclcpp::get_logger(name()), "InletPressureStable: timeout waiting for inlet pressure to stabilise");
        return BT::NodeStatus::FAILURE;
    }


    std::shared_ptr<BoosterTelemetryCache> cache;
    if (!config().blackboard->get("telemetry_cache", cache) || !cache) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "InletPressureStable: telemetry_cache not on blackboard");
        return BT::NodeStatus::FAILURE;
    }

    auto [msg, stamp] = cache->latest();
    if (!msg) { return BT::NodeStatus::RUNNING; }

    auto index_res     = getInput<int>("inlet_pt_index");
    auto samples_res   = getInput<int>("stabilization_samples");
    auto tolerance_res = getInput<double>("stability_tolerance");

    if (!index_res || !samples_res || !tolerance_res) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "InletPressureStable: missing input ports");
        return BT::NodeStatus::FAILURE;
    }

    const int    inlet_pt_index        = index_res.value();
    const int    stabilization_samples = samples_res.value();
    const double stability_tolerance   = tolerance_res.value();

    if (stabilization_samples <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "InletPressureStable: stabilization_samples must be > 0");
        return BT::NodeStatus::FAILURE;
    }

    if (msg->timestamp != last_stamp_) {
        window_.push_back(msg->pt_bar[inlet_pt_index]);
        while (window_.size() > static_cast<std::size_t>(stabilization_samples)) {
            window_.pop_front();
        }
        last_stamp_ = msg->timestamp;
    }

    if (window_.size() < static_cast<std::size_t>(stabilization_samples)) {
        return BT::NodeStatus::RUNNING;
    }

    const auto [min_it, max_it] = std::minmax_element(window_.begin(), window_.end());
    const double variation = *max_it - *min_it;

    if (variation <= stability_tolerance) {
        RCLCPP_INFO(rclcpp::get_logger(name()), "InletPressureStable: stable — variation: %.4f bar", variation);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void InletPressureStable::onHalted() {
    window_.clear();
    last_stamp_ = builtin_interfaces::msg::Time();
}

// ==============================================================================
// WAIT — VFD At Speed
// ==============================================================================

VFDAtSpeed::VFDAtSpeed(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config) {}

BT::PortsList VFDAtSpeed::providedPorts() {
    return {
        BT::InputPort<int>("vfd_index"),
        BT::InputPort<double>("target_speed"),
        BT::InputPort<double>("ramp_tolerance"),
        BT::InputPort<int>("ramp_timeout_ms")
    };
}

BT::NodeStatus VFDAtSpeed::onStart() {
    auto tolerance_res = getInput<double>("ramp_tolerance");
    auto timeout_res   = getInput<int>("ramp_timeout_ms");
    if (!tolerance_res || !timeout_res) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "VFDAtSpeed: missing ramp_tolerance or ramp_timeout_ms port");
        return BT::NodeStatus::FAILURE;
    }
    tolerance_  = tolerance_res.value();
    timeout_    = std::chrono::milliseconds(timeout_res.value());
    start_time_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus VFDAtSpeed::onRunning() {
    if (std::chrono::steady_clock::now() - start_time_ > timeout_) {
        RCLCPP_WARN(rclcpp::get_logger(name()), "VFDAtSpeed: timeout waiting for VFD to reach speed");
        return BT::NodeStatus::FAILURE;
    }

    std::shared_ptr<BoosterTelemetryCache> cache;
    if (!config().blackboard->get("telemetry_cache", cache) || !cache) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "VFDAtSpeed: telemetry_cache not on blackboard");
        return BT::NodeStatus::FAILURE;
    }

    auto [msg, stamp] = cache->latest();
    if (!msg) { return BT::NodeStatus::RUNNING; }

    auto index_res  = getInput<int>("vfd_index");
    auto target_res = getInput<double>("target_speed");
    if (!index_res || !target_res) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "VFDAtSpeed: missing input ports");
        return BT::NodeStatus::FAILURE;
    }

    const double speed  = msg->vfd_speed_rpm[index_res.value()];
    const double target = target_res.value();

    if (std::abs(speed - target) <= tolerance_) {
        RCLCPP_INFO(rclcpp::get_logger(name()), "VFDAtSpeed: at speed — %.1f rpm (target: %.1f ± %.1f)", speed, target, tolerance_);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void VFDAtSpeed::onHalted() {}

// ==============================================================================
// WAIT — VFD Stopped
// ==============================================================================

VFDStopped::VFDStopped(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config) {}

BT::PortsList VFDStopped::providedPorts() {
    return {
        BT::InputPort<int>("vfd_index"),
        BT::InputPort<double>("stop_threshold"),
        BT::InputPort<int>("stop_timeout_ms")
    };
}

BT::NodeStatus VFDStopped::onStart() {
    auto timeout_res = getInput<int>("stop_timeout_ms");
    if (!timeout_res) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "VFDStopped: missing stop_timeout_ms port");
        return BT::NodeStatus::FAILURE;
    }
    timeout_    = std::chrono::milliseconds(timeout_res.value());
    start_time_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus VFDStopped::onRunning() {
    if (std::chrono::steady_clock::now() - start_time_ > timeout_) {
        RCLCPP_WARN(rclcpp::get_logger(name()), "VFDStopped: timeout waiting for VFD to stop");
        return BT::NodeStatus::FAILURE;
    }

    std::shared_ptr<BoosterTelemetryCache> cache;
    if (!config().blackboard->get("telemetry_cache", cache) || !cache) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "VFDStopped: telemetry_cache not on blackboard");
        return BT::NodeStatus::FAILURE;
    }

    auto [msg, stamp] = cache->latest();
    if (!msg) { return BT::NodeStatus::RUNNING; }

    auto index_res     = getInput<int>("vfd_index");
    auto threshold_res = getInput<double>("stop_threshold");
    if (!index_res || !threshold_res) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "VFDStopped: missing input ports");
        return BT::NodeStatus::FAILURE;
    }

    const double  speed     = msg->vfd_speed_rpm[index_res.value()];
    const uint8_t state     = msg->vfd_state[index_res.value()];
    const double  threshold = threshold_res.value();

    using Msg = mserve_interfaces::msg::CompressorTelemetry;
    if (speed <= threshold && state != Msg::VFD_RUNNING) {
        RCLCPP_INFO(rclcpp::get_logger(name()), "VFDStopped: stopped — speed: %.1f rpm, state: %d", speed, state);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void VFDStopped::onHalted() {}

// ==============================================================================
// WAIT — Outlet At Pressure
// ==============================================================================

OutletAtPressure::OutletAtPressure(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config) {}

BT::PortsList OutletAtPressure::providedPorts() {
    return {
        BT::InputPort<int>("outlet_pt_index"),
        BT::InputPort<double>("target_pressure")
    };
}

BT::NodeStatus OutletAtPressure::onStart() {
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OutletAtPressure::onRunning() {
    std::shared_ptr<BoosterTelemetryCache> cache;
    if (!config().blackboard->get("telemetry_cache", cache) || !cache) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "OutletAtPressure: telemetry_cache not on blackboard");
        return BT::NodeStatus::FAILURE;
    }

    auto [msg, stamp] = cache->latest();
    if (!msg) { return BT::NodeStatus::RUNNING; }

    auto index_res  = getInput<int>("outlet_pt_index");
    auto target_res = getInput<double>("target_pressure");
    if (!index_res || !target_res) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "OutletAtPressure: missing input ports");
        return BT::NodeStatus::FAILURE;
    }

    const double pressure = msg->pt_bar[index_res.value()];
    const double target   = target_res.value();

    if (pressure >= target) {
        RCLCPP_INFO(rclcpp::get_logger(name()), "OutletAtPressure: target reached — %.1f bar", pressure);
        return BT::NodeStatus::SUCCESS;
    }

    // TODO(Stage 4): stall detection — FAILURE if pressure makes no progress over a
    // rolling window. Time-based timeout is wrong here (fill time is storage-dependent).
    return BT::NodeStatus::RUNNING;
}

void OutletAtPressure::onHalted() {}

// ==============================================================================
// WAIT — Pressure Below Threshold
// ==============================================================================

PressureBelowThreshold::PressureBelowThreshold(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config) {}

BT::PortsList PressureBelowThreshold::providedPorts() {
    return {
        BT::InputPort<int>("pt_index"),
        BT::InputPort<double>("threshold_bar"),          // explicit threshold; used in stop sequence
        BT::InputPort<double>("reenable_pressure_bar"), // absolute re-engage pressure for HOLD mode
        BT::InputPort<int>("timeout_ms")
    };
}

BT::NodeStatus PressureBelowThreshold::onStart() {
    auto timeout_res = getInput<int>("timeout_ms");
    if (!timeout_res) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "PressureBelowThreshold: missing timeout_ms port");
        return BT::NodeStatus::FAILURE;
    }
    timeout_    = std::chrono::milliseconds(timeout_res.value());
    start_time_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PressureBelowThreshold::onRunning() {
    // timeout_ms=0 means no timeout (used by the START_IDLE maintain loop)
    if (timeout_ != std::chrono::milliseconds(0) &&
        std::chrono::steady_clock::now() - start_time_ > timeout_) {
        RCLCPP_WARN(rclcpp::get_logger(name()), "PressureBelowThreshold: timeout waiting for pressure to drop");
        return BT::NodeStatus::FAILURE;
    }

    std::shared_ptr<BoosterTelemetryCache> cache;
    if (!config().blackboard->get("telemetry_cache", cache) || !cache) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "PressureBelowThreshold: telemetry_cache not on blackboard");
        return BT::NodeStatus::FAILURE;
    }

    auto [msg, stamp] = cache->latest();
    if (!msg) { return BT::NodeStatus::RUNNING; }

    auto index_res = getInput<int>("pt_index");
    if (!index_res) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "PressureBelowThreshold: missing pt_index port");
        return BT::NodeStatus::FAILURE;
    }

    double threshold;
    auto thresh_res = getInput<double>("threshold_bar");
    if (thresh_res) {
        threshold = thresh_res.value();
    } else {
        auto reenable_res = getInput<double>("reenable_pressure_bar");
        if (!reenable_res) {
            RCLCPP_ERROR(rclcpp::get_logger(name()), "PressureBelowThreshold: neither threshold_bar nor reenable_pressure_bar provided");
            return BT::NodeStatus::FAILURE;
        }
        threshold = reenable_res.value();
    }

    const double pressure = msg->pt_bar[index_res.value()];
    if (pressure < threshold) {
        RCLCPP_INFO(rclcpp::get_logger(name()), "PressureBelowThreshold: %.1f bar below %.1f bar", pressure, threshold);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void PressureBelowThreshold::onHalted() {}

// ==============================================================================
// GATE — On Target Is
// ==============================================================================

OnTargetIs::OnTargetIs(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList OnTargetIs::providedPorts() {
    return { BT::InputPort<uint8_t>("value") };
}

BT::NodeStatus OnTargetIs::tick() {
    auto val_res = getInput<uint8_t>("value");
    if (!val_res) return BT::NodeStatus::FAILURE;
    uint8_t actual = 0;
    (void)config().blackboard->get("on_target", actual);
    return (actual == val_res.value()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ==============================================================================
// GUARD — Inlet Guard
// ==============================================================================

InletGuard::InletGuard(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config) {}

BT::PortsList InletGuard::providedPorts() {
    return {
        BT::InputPort<int>("inlet_pt_index"),
        BT::InputPort<uint8_t>("on_inlet_starve"),
        BT::InputPort<double>("inlet_starve_bar"),
        BT::InputPort<double>("inlet_resume_bar"),
    };
}

BT::NodeStatus InletGuard::onStart() {
    if (!config().blackboard->get("telemetry_cache", cache_)) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "InletGuard: telemetry_cache not on blackboard");
        return BT::NodeStatus::FAILURE;
    }
    starving_ = false;
    return onRunning();
}

BT::NodeStatus InletGuard::onRunning() {
    auto idx_res    = getInput<int>("inlet_pt_index");
    auto mode_res   = getInput<uint8_t>("on_inlet_starve");
    auto starve_res = getInput<double>("inlet_starve_bar");
    auto resume_res = getInput<double>("inlet_resume_bar");
    if (!idx_res || !mode_res || !starve_res || !resume_res) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "InletGuard: missing input port");
        return BT::NodeStatus::FAILURE;
    }

    auto [msg, stamp] = cache_->latest();
    if (!msg) {
        // No telemetry — ABORT fails safe; PAUSE assumes healthy until data arrives
        return (mode_res.value() == 0) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }

    const double  pressure  = msg->pt_bar[idx_res.value()];
    const uint8_t mode      = mode_res.value();
    const double  starve_at = starve_res.value();
    const double  resume_at = resume_res.value();

    if (mode == 0) {  // ABORT — instantaneous gate, no state
        return (pressure >= starve_at) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    // PAUSE mode — hysteresis: clear only at resume threshold
    if (!starving_) {
        if (pressure < starve_at) {
            starving_ = true;
            RCLCPP_ERROR(rclcpp::get_logger(name()),
                "InletGuard: inlet STARVED %.1f bar < %.1f bar — pausing compression",
                pressure, starve_at);
            return BT::NodeStatus::RUNNING;
        }
        return BT::NodeStatus::SUCCESS;
    } else {
        if (pressure >= resume_at) {
            starving_ = false;
            RCLCPP_INFO(rclcpp::get_logger(name()),
                "InletGuard: inlet recovered %.1f bar >= %.1f bar — resuming compression",
                pressure, resume_at);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
}

void InletGuard::onHalted() {
    starving_ = false;
}

// ==============================================================================
// GATE — Inlet Pressure Safe
// ==============================================================================

InletPressureSafe::InletPressureSafe(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config) {}

BT::PortsList InletPressureSafe::providedPorts() {
    return {
        BT::InputPort<int>("inlet_pt_index"),
        BT::InputPort<double>("safe_pressure")
    };
}

BT::NodeStatus InletPressureSafe::tick() {
    std::shared_ptr<BoosterTelemetryCache> cache;
    if (!config().blackboard->get("telemetry_cache", cache) || !cache) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "InletPressureSafe: telemetry_cache not on blackboard");
        return BT::NodeStatus::FAILURE;
    }

    auto [msg, stamp] = cache->latest();
    if (!msg) {
        RCLCPP_WARN(rclcpp::get_logger(name()), "InletPressureSafe: no telemetry received yet");
        return BT::NodeStatus::FAILURE;
    }

    auto index_res  = getInput<int>("inlet_pt_index");
    auto target_res = getInput<double>("safe_pressure");
    if (!index_res || !target_res) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "InletPressureSafe: missing input ports");
        return BT::NodeStatus::FAILURE;
    }

    const double pressure = msg->pt_bar[index_res.value()];
    const double target   = target_res.value();

    // RUNNING while safe (parallel monitor), FAILURE to abort startup
    return pressure >= target ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
}

// ==============================================================================
// REPORT — Log Compression Start
// ==============================================================================

LogCompressionStart::LogCompressionStart(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config) {}

BT::PortsList LogCompressionStart::providedPorts() {
    return {
        BT::InputPort<int>("outlet_pt_index"),
        BT::InputPort<double>("target_pressure")
    };
}

BT::NodeStatus LogCompressionStart::tick() {
    std::shared_ptr<BoosterTelemetryCache> cache;
    if (!config().blackboard->get("telemetry_cache", cache) || !cache) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "LogCompressionStart: telemetry_cache not on blackboard");
        return BT::NodeStatus::FAILURE;
    }

    auto index_res  = getInput<int>("outlet_pt_index");
    auto target_res = getInput<double>("target_pressure");
    if (!index_res || !target_res) {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "LogCompressionStart: missing input ports");
        return BT::NodeStatus::FAILURE;
    }

    // Route through the booster node's own logger name so these lines group
    // with the rest of [low_booster]/[high_booster] output (e.g. for Loki).
    std::string ros_node_name;
    const bool have_node_name = config().blackboard->get("ros_node_name", ros_node_name);
    auto logger = have_node_name ? rclcpp::get_logger(ros_node_name) : rclcpp::get_logger(name());

    auto [msg, stamp] = cache->latest();
    if (msg) {
        RCLCPP_INFO(logger, "Compressing from %.1f bar to target %.1f bar",
                    msg->pt_bar[index_res.value()], target_res.value());
    } else {
        RCLCPP_INFO(logger, "Compressing to target %.1f bar", target_res.value());
    }
    return BT::NodeStatus::SUCCESS;
}

} // namespace hyfleet_booster

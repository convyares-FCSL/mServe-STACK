#pragma once

#include <chrono>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <mserve_interfaces/msg/compressor_telemetry.hpp>

namespace hyfleet_booster {

// Forward declaration — full type defined in booster_telemetry_cache.hpp (included by .cpp files)
class BoosterTelemetryCache;

// ==============================================================================
// Classification:
//   Gate  (instantaneous yes/no, SUCCESS/FAILURE only) → BT::ConditionNode
//   Wait  (RUNNING until a state is reached)           → BT::StatefulActionNode
// ==============================================================================

// ==============================================================================
// WAIT — Inlet Pressure Stable
// Accumulates a rolling window; returns SUCCESS once variation <= tolerance.
// Belongs before VFD start to confirm stable supply pressure.
// ==============================================================================

class InletPressureStable : public BT::StatefulActionNode {
    public:
        InletPressureStable(const std::string& name, const BT::NodeConfiguration& config);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart()   override;
        BT::NodeStatus onRunning() override;
        void           onHalted()  override;

    private:
        std::chrono::steady_clock::time_point start_time_;
        std::chrono::milliseconds             timeout_{};
        std::deque<double>                    window_;
        builtin_interfaces::msg::Time         last_stamp_{};
};

// ==============================================================================
// WAIT — VFD At Speed
// Polls until |speed - target| <= ramp_tolerance. Wall-clock timeout → FAILURE.
// ==============================================================================

class VFDAtSpeed : public BT::StatefulActionNode {
    public:
        VFDAtSpeed(const std::string& name, const BT::NodeConfiguration& config);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart()   override;
        BT::NodeStatus onRunning() override;
        void           onHalted()  override;

    private:
        std::chrono::steady_clock::time_point start_time_;
        std::chrono::milliseconds             timeout_{};
        double                                tolerance_{};
};

// ==============================================================================
// WAIT — VFD Stopped
// Polls until speed <= stop_threshold and state != VFD_RUNNING.
// Wall-clock timeout → FAILURE.
// ==============================================================================

class VFDStopped : public BT::StatefulActionNode {
    public:
        VFDStopped(const std::string& name, const BT::NodeConfiguration& config);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart()   override;
        BT::NodeStatus onRunning() override;
        void           onHalted()  override;

    private:
        std::chrono::steady_clock::time_point start_time_;
        std::chrono::milliseconds             timeout_{};
};

// ==============================================================================
// WAIT — Outlet At Pressure
// Polls until outlet pressure >= target_pressure. No wall-clock timeout:
// fill time is storage-dependent; stall detection (Stage 4) is the failure mode.
// ==============================================================================

class OutletAtPressure : public BT::StatefulActionNode {
    public:
        OutletAtPressure(const std::string& name, const BT::NodeConfiguration& config);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart()   override;
        BT::NodeStatus onRunning() override;
        void           onHalted()  override;
};

// ==============================================================================
// WAIT — Pressure Below Threshold
// Polls until pt_bar[pt_index] falls below the threshold. Wall-clock timeout → FAILURE.
// Ports: threshold_bar (explicit) OR reenable_pressure_bar (absolute re-engage pressure).
// ==============================================================================

class PressureBelowThreshold : public BT::StatefulActionNode {
    public:
        PressureBelowThreshold(const std::string& name, const BT::NodeConfiguration& config);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart()   override;
        BT::NodeStatus onRunning() override;
        void           onHalted()  override;

    private:
        std::chrono::steady_clock::time_point start_time_;
        std::chrono::milliseconds             timeout_{};
};

// ==============================================================================
// GATE — On Target Is
// Reads on_target from the blackboard and returns SUCCESS if it equals value,
// FAILURE otherwise. Used in compress_tree.xml Fallback to route SUCCEED vs HOLD.
// ==============================================================================

class OnTargetIs : public BT::ConditionNode {
public:
    OnTargetIs(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// ==============================================================================
// GUARD — Inlet Guard
// ABORT mode (on_inlet_starve=0): instantaneous gate — FAILURE if inlet < starve_bar.
// PAUSE mode (on_inlet_starve=1): stateful hysteresis — RUNNING while starved
//   (clears only when inlet >= resume_bar), SUCCESS when healthy.
//   In a ReactiveSequence ahead of CompressToTarget, RUNNING halts CompressToTarget
//   (PCSV off via onHalted), SUCCESS lets it run. One mechanism for both pause paths.
// ==============================================================================

class InletGuard : public BT::StatefulActionNode {
public:
    InletGuard(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
private:
    std::shared_ptr<BoosterTelemetryCache> cache_;
    bool starving_ = false;
};

// ==============================================================================
// GATE — Inlet Pressure Safe
// Instantaneous check: FAILURE if supply pressure < safe_pressure.
// Aborts startup immediately — correct semantics for a safety monitor.
// ==============================================================================

class InletPressureSafe : public BT::ConditionNode {
    public:
        InletPressureSafe(const std::string& name, const BT::NodeConfiguration& config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;
};

// ==============================================================================
// REPORT — Log Compression Start
// One-shot side effect: logs current outlet pressure vs target when a START
// goal begins. Always returns SUCCESS — purely a log line for operators/Loki.
// ==============================================================================

class LogCompressionStart : public BT::SyncActionNode {
    public:
        LogCompressionStart(const std::string& name, const BT::NodeConfiguration& config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;
};

} // namespace hyfleet_booster

#pragma once

#include <chrono>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <mserve_interfaces/msg/compressor_telemetry.hpp>

namespace hyfleet_booster {

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
// Polls until pt_bar[pt_index] < threshold_bar. Wall-clock timeout → FAILURE.
// Used in stop sequence to confirm hydraulic lines de-pressurised.
// Also used in START_IDLE maintain loop to detect outlet droop below re-enable band.
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

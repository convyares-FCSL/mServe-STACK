#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/timer.hpp>

#include "interfaces/action/control_compressor.hpp"
#include "internal/booster_manager.hpp"
#include "internal/safety_monitor.hpp"

namespace compressor
{

class CompressorAction;
class DiagnosticsPublisher;
class DeviceDispatcher;
class TelemetrySubscriber;
struct CompressorTelemetryUpdate;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CompressorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CompressorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CompressorNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void process_pending_control();
  void handle_telemetry(const CompressorTelemetryUpdate & telemetry);
  void publish_diagnostics(
    const CompressorStatus & status,
    bool telemetry_received,
    uint8_t telemetry_mode,
    const builtin_interfaces::msg::Time & telemetry_timestamp) const;
  bool dispatch_safe_stop(const std::string & reason);
  void reset_ros_entities();

  std::mutex booster_manager_mutex_;
  bool telemetry_received_ = false;
  uint8_t latest_telemetry_mode_ = 0;
  builtin_interfaces::msg::Time latest_telemetry_timestamp_;
  rclcpp::Time latest_telemetry_receive_time_;
  double telemetry_timeout_sec_ = 2.0;
  SafetyStatus latest_safety_status_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
  rclcpp::CallbackGroup::SharedPtr telemetry_callback_group_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::unique_ptr<CompressorAction> compressor_action_;
  std::unique_ptr<DiagnosticsPublisher> diagnostics_publisher_;
  std::unique_ptr<BoosterManager> booster_manager_;
  std::unique_ptr<DeviceDispatcher> device_dispatcher_;
  std::unique_ptr<SafetyMonitor> safety_monitor_;
  std::unique_ptr<TelemetrySubscriber> telemetry_subscriber_;
};

}  // namespace compressor

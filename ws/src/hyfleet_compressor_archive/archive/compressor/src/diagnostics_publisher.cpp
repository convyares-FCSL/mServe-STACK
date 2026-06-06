#include "internal/diagnostics_publisher.hpp"

#include <utility>

namespace compressor
{

DiagnosticsPublisher::DiagnosticsPublisher(
  rclcpp_lifecycle::LifecycleNode & node,
  std::string topic_name)
: node_(node),
  topic_name_(std::move(topic_name))
{
}

void DiagnosticsPublisher::configure()
{
  publisher_ =
    node_.create_publisher<interfaces::msg::CompressorDiagnostics>(topic_name_, 10);
}

void DiagnosticsPublisher::activate()
{
  if (publisher_) {
    publisher_->on_activate();
  }
}

void DiagnosticsPublisher::deactivate()
{
  if (publisher_) {
    publisher_->on_deactivate();
  }
}

void DiagnosticsPublisher::reset()
{
  publisher_.reset();
}

void DiagnosticsPublisher::publish(
  const CompressorStatus & status,
  bool telemetry_received,
  uint8_t telemetry_mode,
  const builtin_interfaces::msg::Time & telemetry_timestamp) const
{
  if (!publisher_ || !publisher_->is_activated()) {
    return;
  }

  interfaces::msg::CompressorDiagnostics diagnostics;
  diagnostics.timestamp = node_.now();
  diagnostics.telemetry_timestamp = telemetry_timestamp;
  diagnostics.telemetry_received = telemetry_received;
  diagnostics.telemetry_mode = telemetry_mode;
  diagnostics.low_booster = make_booster_diagnostics(status.low_booster);
  diagnostics.high_booster = make_booster_diagnostics(status.high_booster);

  publisher_->publish(diagnostics);
}

interfaces::msg::BoosterDiagnostics DiagnosticsPublisher::make_booster_diagnostics(
  const BoosterStatus & status)
{
  interfaces::msg::BoosterDiagnostics diagnostics;
  diagnostics.name = status.name;
  diagnostics.active = status.active;
  diagnostics.target_reached = status.target_reached;
  diagnostics.phase = static_cast<uint8_t>(status.phase);
  diagnostics.status_message = status.message;
  diagnostics.target_pressure_bar = status.target_pressure;
  diagnostics.input_pressure_bar = status.telemetry.input_pressure;
  diagnostics.output_pressure_bar = status.telemetry.output_pressure;
  diagnostics.compression_ratio = status.telemetry.compression_ratio;
  diagnostics.input_temperature_celsius = status.telemetry.input_temperature;
  diagnostics.output_temperature_celsius = status.telemetry.output_temperature;
  diagnostics.vfd_energy_kj = status.telemetry.vfd_energy;
  diagnostics.vfd_speed_rpm = status.telemetry.vfd_speed;
  diagnostics.pcsv_cpm = status.cpm;
  diagnostics.hours_run = status.hours_run;
  return diagnostics;
}

}  // namespace compressor

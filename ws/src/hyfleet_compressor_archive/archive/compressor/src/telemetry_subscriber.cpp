#include "internal/telemetry_subscriber.hpp"

#include <utility>

namespace compressor
{

TelemetrySubscriber::TelemetrySubscriber(
  rclcpp_lifecycle::LifecycleNode & node,
  std::string topic_name,
  BoosterTelemetryMapping low_booster_mapping,
  BoosterTelemetryMapping high_booster_mapping,
  Callback callback)
: low_booster_mapping_(std::move(low_booster_mapping)),
  high_booster_mapping_(std::move(high_booster_mapping)),
  callback_(std::move(callback)),
  node_(node),
  topic_name_(std::move(topic_name))
{
}

void TelemetrySubscriber::configure(rclcpp::CallbackGroup::SharedPtr callback_group)
{
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group;

  subscription_ = node_.create_subscription<interfaces::msg::CompressorTelemetry>(
    topic_name_,
    10,
    [this](const interfaces::msg::CompressorTelemetry::SharedPtr msg) {
      handle_message(msg);
    },
    options);
}

void TelemetrySubscriber::reset()
{
  subscription_.reset();
}

void TelemetrySubscriber::handle_message(
  const interfaces::msg::CompressorTelemetry::SharedPtr msg) const
{
  if (!callback_) {
    return;
  }

  CompressorTelemetryUpdate update;
  update.mode = msg->mode;
  update.timestamp = msg->timestamp;
  update.hpu_temperature = msg->hpu_tt_celsius;
  for (size_t index = 0; index < update.hbu_temperatures.size(); ++index) {
    update.hbu_temperatures[index] = msg->hbu_tt_celsius[index];
  }
  update.low_booster = booster_telemetry_from_msg(*msg, low_booster_mapping_);
  update.high_booster = booster_telemetry_from_msg(*msg, high_booster_mapping_);
  callback_(update);
}

BoosterTelemetry TelemetrySubscriber::booster_telemetry_from_msg(
  const interfaces::msg::CompressorTelemetry & msg,
  const BoosterTelemetryMapping & mapping) const
{
  BoosterTelemetry telemetry;
  telemetry.input_pressure = msg.hbu_pt_bar[mapping.inlet_pressure_index];
  telemetry.output_pressure = msg.hbu_pt_bar[mapping.outlet_pressure_index];
  telemetry.input_temperature = msg.hbu_tt_celsius[mapping.inlet_temperature_index];
  telemetry.output_temperature = msg.hbu_tt_celsius[mapping.outlet_temperature_index];

  if (mapping.vfd_index == 1) {
    telemetry.vfd_energy = msg.vfd1_energy_kj;
    telemetry.vfd_speed = msg.vfd1_speed_rpm;
  } else {
    telemetry.vfd_energy = msg.vfd2_energy_kj;
    telemetry.vfd_speed = msg.vfd2_speed_rpm;
  }

  if (telemetry.input_pressure > 0.0) {
    telemetry.compression_ratio = telemetry.output_pressure / telemetry.input_pressure;
  }

  return telemetry;
}

}  // namespace compressor

#pragma once

#include <array>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "builtin_interfaces/msg/time.hpp"
#include "interfaces/msg/compressor_telemetry.hpp"

#include "internal/compressor_config.hpp"
#include "internal/compressor_types.hpp"

namespace compressor
{

struct CompressorTelemetryUpdate
{
  uint8_t mode = 0;
  builtin_interfaces::msg::Time timestamp;
  std::array<double, 12> hbu_temperatures{};
  double hpu_temperature = 0.0;
  BoosterTelemetry low_booster;
  BoosterTelemetry high_booster;
};

class TelemetrySubscriber
{
public:
  using Callback = std::function<void(const CompressorTelemetryUpdate &)>;

  TelemetrySubscriber(
    rclcpp_lifecycle::LifecycleNode & node,
    std::string topic_name,
    BoosterTelemetryMapping low_booster_mapping,
    BoosterTelemetryMapping high_booster_mapping,
    Callback callback);

  void configure(rclcpp::CallbackGroup::SharedPtr callback_group = nullptr);
  void reset();

private:
  void handle_message(const interfaces::msg::CompressorTelemetry::SharedPtr msg) const;
  BoosterTelemetry booster_telemetry_from_msg(
    const interfaces::msg::CompressorTelemetry & msg,
    const BoosterTelemetryMapping & mapping) const;

  BoosterTelemetryMapping low_booster_mapping_;
  BoosterTelemetryMapping high_booster_mapping_;
  Callback callback_;
  rclcpp_lifecycle::LifecycleNode & node_;
  std::string topic_name_;
  rclcpp::Subscription<interfaces::msg::CompressorTelemetry>::SharedPtr subscription_;
};

}  // namespace compressor

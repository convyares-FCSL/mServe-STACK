#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "builtin_interfaces/msg/time.hpp"
#include "interfaces/msg/booster_diagnostics.hpp"
#include "interfaces/msg/compressor_diagnostics.hpp"

#include "internal/compressor_types.hpp"

namespace compressor
{

class DiagnosticsPublisher
{
public:
  DiagnosticsPublisher(rclcpp_lifecycle::LifecycleNode & node, std::string topic_name);

  void configure();
  void activate();
  void deactivate();
  void reset();

  void publish(
    const CompressorStatus & status,
    bool telemetry_received,
    uint8_t telemetry_mode,
    const builtin_interfaces::msg::Time & telemetry_timestamp) const;

private:
  static interfaces::msg::BoosterDiagnostics make_booster_diagnostics(
    const BoosterStatus & status);

  rclcpp_lifecycle::LifecycleNode & node_;
  std::string topic_name_;
  rclcpp_lifecycle::LifecyclePublisher<
    interfaces::msg::CompressorDiagnostics>::SharedPtr publisher_;
};

}  // namespace compressor

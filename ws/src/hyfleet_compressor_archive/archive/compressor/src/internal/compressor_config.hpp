#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "internal/compressor_types.hpp"
#include "internal/device_types.hpp"

namespace compressor
{

struct BoosterTelemetryMapping
{
  size_t inlet_pressure_index = 0;
  size_t outlet_pressure_index = 0;
  size_t inlet_temperature_index = 0;
  size_t outlet_temperature_index = 0;
  size_t vfd_index = 1;
};

struct CompressorRosConfig
{
  std::string telemetry_topic;
  std::string diagnostics_topic;
  std::string control_action_name;
};

struct CommandLimitsConfig
{
  double min_pressure_bar = 0.0;
  double max_pressure_bar = 700.0;
  double low_booster_min_pressure_bar = 0.0;
  double low_booster_max_pressure_bar = 700.0;
  double high_booster_min_pressure_bar = 0.0;
  double high_booster_max_pressure_bar = 700.0;
};

struct SafetyConfig
{
  std::vector<uint8_t> heater_ids{1, 2};
  double telemetry_timeout_sec = 2.0;
  double oil_temp_inhibit_high_celsius = 60.0;
  double oil_temp_inhibit_low_celsius = 15.0;
  double hbu_inlet_temp_limit_celsius = 50.0;
  double hbu_outlet_temp_limit_celsius = 190.0;
  double chiller_outlet_temp_limit_celsius = 70.0;
  std::vector<size_t> inlet_temp_indices;
  std::vector<size_t> outlet_temp_indices;
  std::vector<size_t> chiller_temp_indices;
};

struct CompressorConfig
{
  CompressorRosConfig ros;
  CommandLimitsConfig command_limits;
  SafetyConfig safety;
  DeviceDispatcherConfig device_dispatcher;
  BoosterConfig low_booster;
  BoosterConfig high_booster;
  BoosterTelemetryMapping low_booster_telemetry;
  BoosterTelemetryMapping high_booster_telemetry;
};

CompressorConfig load_compressor_config(rclcpp_lifecycle::LifecycleNode & node);

}  // namespace compressor

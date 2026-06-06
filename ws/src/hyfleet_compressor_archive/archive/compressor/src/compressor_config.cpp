#include "internal/compressor_config.hpp"

#include "interfaces/msg/compressor_telemetry.hpp"

#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace compressor
{

namespace
{
size_t hbu_pressure_count()
{
  return std::tuple_size<interfaces::msg::CompressorTelemetry::_hbu_pt_bar_type>::value;
}

size_t hbu_temperature_count()
{
  return std::tuple_size<interfaces::msg::CompressorTelemetry::_hbu_tt_celsius_type>::value;
}

std::string declare_or_get_string(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & name,
  const std::string & value)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<std::string>(name, value);
  }
  return node.get_parameter(name).as_string();
}

double declare_or_get_double(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & name,
  double value)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<double>(name, value);
  }
  return node.get_parameter(name).as_double();
}

int64_t declare_or_get_int(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & name,
  int64_t value)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<int64_t>(name, value);
  }
  return node.get_parameter(name).as_int();
}

std::vector<int64_t> declare_or_get_int_array(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & name,
  const std::vector<int64_t> & value)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<std::vector<int64_t>>(name, value);
  }
  return node.get_parameter(name).as_integer_array();
}

size_t require_index(const std::string & name, int64_t value, size_t size)
{
  if (value < 0 || static_cast<size_t>(value) >= size) {
    throw std::runtime_error(name + " index is outside telemetry array bounds");
  }
  return static_cast<size_t>(value);
}

size_t require_vfd_index(const std::string & name, int64_t value)
{
  if (value != 1 && value != 2) {
    throw std::runtime_error(name + " must be 1 or 2");
  }
  return static_cast<size_t>(value);
}

std::vector<size_t> load_indices(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & name,
  const std::vector<int64_t> & defaults,
  size_t size)
{
  std::vector<size_t> indices;
  for (const auto value : declare_or_get_int_array(node, name, defaults)) {
    indices.push_back(require_index(name, value, size));
  }
  return indices;
}

int32_t require_positive_command_id(const std::string & name, int64_t value)
{
  if (value <= 0 || value > std::numeric_limits<int32_t>::max()) {
    throw std::runtime_error(name + " must be a positive int32");
  }
  return static_cast<int32_t>(value);
}

void require_pressure_limits(
  const std::string & min_name,
  double min_pressure,
  const std::string & max_name,
  double max_pressure)
{
  if (!std::isfinite(min_pressure) || min_pressure < 0.0) {
    throw std::runtime_error(min_name + " must be finite and at least 0");
  }
  if (!std::isfinite(max_pressure)) {
    throw std::runtime_error(max_name + " must be finite");
  }
  if (max_pressure <= min_pressure) {
    throw std::runtime_error(max_name + " must exceed " + min_name);
  }
}

std::vector<uint8_t> load_heater_ids(rclcpp_lifecycle::LifecycleNode & node)
{
  std::vector<uint8_t> heater_ids;
  for (const auto value : declare_or_get_int_array(node, "heater_ids", {1, 2})) {
    if (value <= 0 || value > std::numeric_limits<uint8_t>::max()) {
      throw std::runtime_error("heater_ids must contain positive uint8 values");
    }
    heater_ids.push_back(static_cast<uint8_t>(value));
  }
  if (heater_ids.empty()) {
    throw std::runtime_error("heater_ids must not be empty");
  }
  return heater_ids;
}

BoosterConfig load_booster_config(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & prefix,
  const std::string & default_name,
  const std::string & default_inlet_sv_id,
  const std::string & default_hpu_sv_id,
  const std::string & default_vfd_id,
  const std::string & default_pcsv_id)
{
  BoosterConfig config;
  config.name = declare_or_get_string(node, prefix + "_name", default_name);
  config.inlet_sv_id = declare_or_get_string(node, prefix + "_inlet_sv_id", default_inlet_sv_id);
  config.hpu_sv_id = declare_or_get_string(node, prefix + "_hpu_sv_id", default_hpu_sv_id);
  config.vfd_id = declare_or_get_string(node, prefix + "_vfd_id", default_vfd_id);
  config.pcsv_id = declare_or_get_string(node, prefix + "_pcsv_id", default_pcsv_id);
  config.off_inlet_pressure = declare_or_get_double(node, prefix + "_off_inlet_pressure", 0.2);
  config.target_deadband = declare_or_get_double(node, prefix + "_target_deadband", 0.5);
  config.vfd_target_speed = declare_or_get_double(node, prefix + "_vfd_target_speed", 1000.0);
  config.vfd_ramp_tolerance =
    declare_or_get_double(node, prefix + "_vfd_ramp_tolerance", 25.0);
  config.vfd_stop_threshold =
    declare_or_get_double(node, prefix + "_vfd_stop_threshold", 25.0);
  config.stability_tolerance = declare_or_get_double(node, prefix + "_stability_tolerance", 0.05);
  config.pcsv_cpm = declare_or_get_double(node, prefix + "_pcsv_cpm", 10.0);
  const auto stabilization_sample_count =
    declare_or_get_int(node, prefix + "_stabilization_sample_count", 3);
  const auto vfd_delay_cycles = declare_or_get_int(node, prefix + "_vfd_delay_cycles", 1);
  const auto wait_stabilization_cycles =
    declare_or_get_int(node, prefix + "_wait_stabilization_cycles", 3);

  if (stabilization_sample_count <= 0) {
    throw std::runtime_error(prefix + "_stabilization_sample_count must be positive");
  }
  if (vfd_delay_cycles < 0) {
    throw std::runtime_error(prefix + "_vfd_delay_cycles must not be negative");
  }
  if (wait_stabilization_cycles < 0) {
    throw std::runtime_error(prefix + "_wait_stabilization_cycles must not be negative");
  }

  config.stabilization_sample_count = static_cast<uint32_t>(stabilization_sample_count);
  config.vfd_delay_cycles = static_cast<uint32_t>(vfd_delay_cycles);
  config.wait_stabilization_cycles = static_cast<uint32_t>(wait_stabilization_cycles);
  return config;
}

BoosterTelemetryMapping load_telemetry_mapping(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & prefix,
  int64_t default_inlet_pressure_index,
  int64_t default_outlet_pressure_index,
  int64_t default_vfd_index)
{
  BoosterTelemetryMapping mapping;
  mapping.inlet_pressure_index = require_index(
    prefix + "_inlet_pt_index",
    declare_or_get_int(node, prefix + "_inlet_pt_index", default_inlet_pressure_index),
    hbu_pressure_count());
  mapping.outlet_pressure_index = require_index(
    prefix + "_outlet_pt_index",
    declare_or_get_int(node, prefix + "_outlet_pt_index", default_outlet_pressure_index),
    hbu_pressure_count());
  mapping.inlet_temperature_index = require_index(
    prefix + "_inlet_tt_index",
    declare_or_get_int(node, prefix + "_inlet_tt_index", 0),
    hbu_temperature_count());
  mapping.outlet_temperature_index = require_index(
    prefix + "_outlet_tt_index",
    declare_or_get_int(node, prefix + "_outlet_tt_index", 0),
    hbu_temperature_count());
  mapping.vfd_index = require_vfd_index(
    prefix + "_vfd_index",
    declare_or_get_int(node, prefix + "_vfd_index", default_vfd_index));
  return mapping;
}

}  // namespace

CompressorConfig load_compressor_config(rclcpp_lifecycle::LifecycleNode & node)
{
  CompressorConfig config;
  config.ros.telemetry_topic =
    declare_or_get_string(node, "telemetry_topic", "compressor_telemetry");
  config.ros.diagnostics_topic =
    declare_or_get_string(node, "diagnostics_topic", "compressor_diagnostics");
  config.ros.control_action_name =
    declare_or_get_string(node, "control_action_name", "control_compressor");

  config.device_dispatcher.services.control_low_booster_service_name =
    declare_or_get_string(node, "control_low_booster_service_name", "control_booster_1");
  config.device_dispatcher.services.control_high_booster_service_name =
    declare_or_get_string(node, "control_high_booster_service_name", "control_booster_2");
  config.device_dispatcher.services.control_compressor_service_name =
    declare_or_get_string(node, "control_compressor_service_name", "control_compressor");
  config.device_dispatcher.command_ids.low_booster = require_positive_command_id(
    "low_booster_command_id",
    declare_or_get_int(node, "low_booster_command_id", 1));
  config.device_dispatcher.command_ids.high_booster = require_positive_command_id(
    "high_booster_command_id",
    declare_or_get_int(node, "high_booster_command_id", 2));
  config.device_dispatcher.command_ids.compressor = require_positive_command_id(
    "compressor_command_id",
    declare_or_get_int(node, "compressor_command_id", 1));
  config.device_dispatcher.command_ids.heater_ids = load_heater_ids(node);
  config.command_limits.min_pressure_bar =
    declare_or_get_double(node, "min_pressure_bar", 0.0);
  config.command_limits.max_pressure_bar =
    declare_or_get_double(node, "max_pressure_bar", 700.0);
  config.command_limits.low_booster_min_pressure_bar =
    declare_or_get_double(
    node,
    "low_booster_min_pressure_bar",
    config.command_limits.min_pressure_bar);
  config.command_limits.low_booster_max_pressure_bar =
    declare_or_get_double(
    node,
    "low_booster_max_pressure_bar",
    config.command_limits.max_pressure_bar);
  config.command_limits.high_booster_min_pressure_bar =
    declare_or_get_double(
    node,
    "high_booster_min_pressure_bar",
    config.command_limits.min_pressure_bar);
  config.command_limits.high_booster_max_pressure_bar =
    declare_or_get_double(
    node,
    "high_booster_max_pressure_bar",
    config.command_limits.max_pressure_bar);
  config.device_dispatcher.limits.pcsv_min_cpm =
    declare_or_get_double(node, "pcsv_min_cpm", 0.0);
  config.device_dispatcher.limits.pcsv_max_cpm =
    declare_or_get_double(node, "pcsv_max_cpm", 120.0);
  config.device_dispatcher.limits.vfd_min_speed_rpm =
    declare_or_get_double(node, "vfd_min_speed_rpm", 0.0);
  config.device_dispatcher.limits.vfd_max_speed_rpm =
    declare_or_get_double(node, "vfd_max_speed_rpm", 6000.0);
  config.device_dispatcher.command_ack_timeout_sec =
    declare_or_get_double(node, "command_ack_timeout_sec", 1.0);

  require_pressure_limits(
    "min_pressure_bar",
    config.command_limits.min_pressure_bar,
    "max_pressure_bar",
    config.command_limits.max_pressure_bar);
  require_pressure_limits(
    "low_booster_min_pressure_bar",
    config.command_limits.low_booster_min_pressure_bar,
    "low_booster_max_pressure_bar",
    config.command_limits.low_booster_max_pressure_bar);
  require_pressure_limits(
    "high_booster_min_pressure_bar",
    config.command_limits.high_booster_min_pressure_bar,
    "high_booster_max_pressure_bar",
    config.command_limits.high_booster_max_pressure_bar);
  if (!std::isfinite(config.device_dispatcher.limits.pcsv_min_cpm) ||
    config.device_dispatcher.limits.pcsv_min_cpm < 0.0)
  {
    throw std::runtime_error("pcsv_min_cpm must be finite and at least 0");
  }
  if (!std::isfinite(config.device_dispatcher.limits.pcsv_max_cpm)) {
    throw std::runtime_error("pcsv_max_cpm must be finite");
  }
  if (config.device_dispatcher.limits.pcsv_max_cpm <=
    config.device_dispatcher.limits.pcsv_min_cpm)
  {
    throw std::runtime_error("pcsv_max_cpm must exceed pcsv_min_cpm");
  }
  if (!std::isfinite(config.device_dispatcher.limits.vfd_min_speed_rpm) ||
    config.device_dispatcher.limits.vfd_min_speed_rpm < 0.0)
  {
    throw std::runtime_error("vfd_min_speed_rpm must be finite and at least 0");
  }
  if (!std::isfinite(config.device_dispatcher.limits.vfd_max_speed_rpm)) {
    throw std::runtime_error("vfd_max_speed_rpm must be finite");
  }
  if (config.device_dispatcher.limits.vfd_max_speed_rpm <=
    config.device_dispatcher.limits.vfd_min_speed_rpm)
  {
    throw std::runtime_error("vfd_max_speed_rpm must exceed vfd_min_speed_rpm");
  }
  if (!std::isfinite(config.device_dispatcher.command_ack_timeout_sec) ||
    config.device_dispatcher.command_ack_timeout_sec < 0.001 ||
    config.device_dispatcher.command_ack_timeout_sec > 60.0)
  {
    throw std::runtime_error("command_ack_timeout_sec must be between 0.001 and 60.0");
  }

  config.low_booster = load_booster_config(
    node,
    "low_booster",
    "low booster",
    "hbu_inlet_1",
    "hpu_sv_1",
    "booster1_vfd",
    "booster1_pcsv");
  config.high_booster = load_booster_config(
    node,
    "high_booster",
    "high booster",
    "hbu_outlet_2",
    "hpu_sv_2",
    "booster2_vfd",
    "booster2_pcsv");

  config.device_dispatcher.low_booster.inlet_sv_id = config.low_booster.inlet_sv_id;
  config.device_dispatcher.low_booster.hpu_sv_id = config.low_booster.hpu_sv_id;
  config.device_dispatcher.low_booster.vfd_id = config.low_booster.vfd_id;
  config.device_dispatcher.low_booster.pcsv_id = config.low_booster.pcsv_id;
  config.device_dispatcher.high_booster.inlet_sv_id = config.high_booster.inlet_sv_id;
  config.device_dispatcher.high_booster.hpu_sv_id = config.high_booster.hpu_sv_id;
  config.device_dispatcher.high_booster.vfd_id = config.high_booster.vfd_id;
  config.device_dispatcher.high_booster.pcsv_id = config.high_booster.pcsv_id;

  config.low_booster_telemetry = load_telemetry_mapping(node, "low_booster", 0, 7, 1);
  config.high_booster_telemetry = load_telemetry_mapping(node, "high_booster", 1, 2, 2);

  config.safety.heater_ids = config.device_dispatcher.command_ids.heater_ids;
  config.safety.telemetry_timeout_sec =
    declare_or_get_double(node, "telemetry_timeout_sec", 2.0);
  config.safety.oil_temp_inhibit_high_celsius =
    declare_or_get_double(node, "oil_temp_inhibit_high_celsius", 60.0);
  config.safety.oil_temp_inhibit_low_celsius =
    declare_or_get_double(node, "oil_temp_inhibit_low_celsius", 15.0);
  config.safety.hbu_inlet_temp_limit_celsius =
    declare_or_get_double(node, "hbu_inlet_temp_limit_celsius", 50.0);
  config.safety.hbu_outlet_temp_limit_celsius =
    declare_or_get_double(node, "hbu_outlet_temp_limit_celsius", 190.0);
  config.safety.chiller_outlet_temp_limit_celsius =
    declare_or_get_double(node, "chiller_outlet_temp_limit_celsius", 70.0);
  config.safety.inlet_temp_indices =
    load_indices(
      node,
      "inlet_temp_indices",
    {0, 1, 4, 5},
      hbu_temperature_count());
  config.safety.outlet_temp_indices =
    load_indices(
      node,
      "outlet_temp_indices",
    {2, 3, 6, 7},
      hbu_temperature_count());
  config.safety.chiller_temp_indices =
    load_indices(
      node,
      "chiller_temp_indices",
    {8, 9},
      hbu_temperature_count());
  if (!std::isfinite(config.safety.telemetry_timeout_sec) ||
    config.safety.telemetry_timeout_sec < 0.001 ||
    config.safety.telemetry_timeout_sec > 60.0)
  {
    throw std::runtime_error("telemetry_timeout_sec must be between 0.001 and 60.0");
  }
  if (!std::isfinite(config.safety.oil_temp_inhibit_low_celsius)) {
    throw std::runtime_error("oil_temp_inhibit_low_celsius must be finite");
  }
  if (!std::isfinite(config.safety.oil_temp_inhibit_high_celsius)) {
    throw std::runtime_error("oil_temp_inhibit_high_celsius must be finite");
  }
  if (config.safety.oil_temp_inhibit_low_celsius >=
    config.safety.oil_temp_inhibit_high_celsius)
  {
    throw std::runtime_error(
      "oil_temp_inhibit_low_celsius must be lower than oil_temp_inhibit_high_celsius");
  }
  if (!std::isfinite(config.safety.hbu_inlet_temp_limit_celsius)) {
    throw std::runtime_error("hbu_inlet_temp_limit_celsius must be finite");
  }
  if (!std::isfinite(config.safety.hbu_outlet_temp_limit_celsius)) {
    throw std::runtime_error("hbu_outlet_temp_limit_celsius must be finite");
  }
  if (!std::isfinite(config.safety.chiller_outlet_temp_limit_celsius)) {
    throw std::runtime_error("chiller_outlet_temp_limit_celsius must be finite");
  }

  return config;
}

}  // namespace compressor

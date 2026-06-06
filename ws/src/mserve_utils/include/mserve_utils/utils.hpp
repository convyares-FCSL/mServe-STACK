#ifndef MSERVE_UTILS_UTILS_HPP
#define MSERVE_UTILS_UTILS_HPP

#include <cmath>
#include <cstdint>
#include <string>

#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

namespace mserve_utils {

// Parameter descriptor helper for bounded integer params.
// Useful for hardware indexes, enum-like integer config, array indexes, etc.

inline rcl_interfaces::msg::ParameterDescriptor make_int_range_descriptor(
  const std::string & description,
  int64_t min_value,
  int64_t max_value,
  int64_t step = 1)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;

  rcl_interfaces::msg::IntegerRange range;
  range.from_value = min_value;
  range.to_value = max_value;
  range.step = step;

  descriptor.integer_range.push_back(range);
  return descriptor;
}

// Parameter descriptor helper for bounded floats params.
// Useful for htemperature and pressures

inline rcl_interfaces::msg::ParameterDescriptor make_double_range_descriptor(
  const std::string & description,
  double min_value,
  double max_value,
  double step = 0.0)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;

  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = min_value;
  range.to_value = max_value;
  range.step = step;

  descriptor.floating_point_range.push_back(range);
  return descriptor;
}

// Tolerant param reader: declares with default if not yet declared, warns when
// the default is being used without an explicit override in the launch config.

inline std::string get_or_declare_param(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger,
  const std::string & name,
  const std::string & default_value,
  const std::string & label)
{
  if (!params.has_parameter(name)) {
    params.declare_parameter(name, rclcpp::ParameterValue(default_value));
  }

  std::string value;
  try {
    value = params.get_parameter(name).as_string();
  } catch (const rclcpp::exceptions::InvalidParameterTypeException &) {
    RCLCPP_WARN(logger, "[config] '%s' has wrong type. Using default.", name.c_str());
    value = default_value;
  }

  auto overrides = params.get_parameter_overrides();
  if (value == default_value && overrides.find(name) == overrides.end()) {
    RCLCPP_WARN(logger, "[config] Using default %s: %s='%s'",
      label.c_str(), name.c_str(), value.c_str());
  }

  return value;
}

inline double get_or_declare_param(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger,
  const std::string & name,
  double default_value,
  const std::string & label)
{
  if (!params.has_parameter(name)) {
    params.declare_parameter(name, rclcpp::ParameterValue(default_value));
  }

  double value;
  try {
    value = params.get_parameter(name).as_double();
  } catch (const rclcpp::exceptions::InvalidParameterTypeException &) {
    RCLCPP_WARN(logger, "[config] '%s' has wrong type. Using default.", name.c_str());
    value = default_value;
  }

  auto overrides = params.get_parameter_overrides();
  if (std::abs(value - default_value) < 1e-9 && overrides.find(name) == overrides.end()) {
    RCLCPP_WARN(logger, "[config] Using default %s: %s='%.3f'",
      label.c_str(), name.c_str(), value);
  }

  return value;
}

inline int get_or_declare_param(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger,
  const std::string & name,
  int default_value,
  const std::string & label)
{
  if (!params.has_parameter(name)) {
    params.declare_parameter(name, rclcpp::ParameterValue(default_value));
  }

  int value;
  try {
    value = params.get_parameter(name).as_int();
  } catch (const rclcpp::exceptions::InvalidParameterTypeException &) {
    RCLCPP_WARN(logger, "[config] '%s' has wrong type. Using default.", name.c_str());
    value = default_value;
  }

  auto overrides = params.get_parameter_overrides();
  if (value == default_value && overrides.find(name) == overrides.end()) {
    RCLCPP_WARN(logger, "[config] Using default %s: %s='%d'",
      label.c_str(), name.c_str(), value);
  }

  return value;
}

}  // namespace mserve_utils

#endif  // MSERVE_UTILS_UTILS_HPP
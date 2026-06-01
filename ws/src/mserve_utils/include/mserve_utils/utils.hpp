#ifndef MSERVE_UTILS_UTILS_HPP
#define MSERVE_UTILS_UTILS_HPP

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

namespace mserve_utils {

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

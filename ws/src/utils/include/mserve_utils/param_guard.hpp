#ifndef MSERVE_UTILS_PARAM_GUARD_HPP
#define MSERVE_UTILS_PARAM_GUARD_HPP

#include <optional>
#include <string>

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace mserve_utils {

// ==============================================================================
// Parameter descriptor builders.
// ==============================================================================

inline rcl_interfaces::msg::ParameterDescriptor bounded_double(
  const std::string & description, double min, double max)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = description;
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = min;
  range.to_value   = max;
  range.step       = 0.0;
  desc.floating_point_range = {range};
  return desc;
}

// ==============================================================================
// Parameter validation result.
// ==============================================================================

template <typename T>
struct ParamValidation {
  bool ok;
  std::optional<T> value;
  std::string reason;

  static ParamValidation<T> success(T val) { return {true, val, ""}; }
  static ParamValidation<T> failure(std::string msg) { return {false, std::nullopt, std::move(msg)}; }
};

}  // namespace mserve_utils

#endif  // MSERVE_UTILS_PARAM_GUARD_HPP

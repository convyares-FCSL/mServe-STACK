#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace hyfleet_compressor
{

struct CommandLimitsConfig
{
  double min_pressure_bar = 0.0;
  double max_pressure_bar = 700.0;
  double low_booster_min_pressure_bar = 0.0;
  double low_booster_max_pressure_bar = 700.0;
  double high_booster_min_pressure_bar = 0.0;
  double high_booster_max_pressure_bar = 700.0;
};

}  // namespace hyfleet_compressor

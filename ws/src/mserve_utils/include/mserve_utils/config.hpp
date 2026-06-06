#ifndef MSERVE_UTILS_CONFIG_HPP
#define MSERVE_UTILS_CONFIG_HPP

#include <string>

namespace mserve_utils {

// ==============================================================================
// mServe configuration parameters.
// ==============================================================================

  // Fixed booster pressure limits
  constexpr double system_pressure_min_bar = 0.0;
  constexpr double system_pressure_max_bar = 950.0;

  // Fixed booster temperature limits (PT100)
  constexpr double pt100_min = -200.0;
  constexpr double pt100_max = 850.0;

  // Fixed sensible timing (ms)
  constexpr double timing_min = 0.0;
  constexpr double timing_max = 30000.0;

}  // namespace mserve_utils

#endif  // MSERVE_UTILS_CONFIG_HPP

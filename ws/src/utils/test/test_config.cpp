#include <gtest/gtest.h>
#include "mserve_utils/config.hpp"

TEST(TestConfig, PressureLimits) {
  EXPECT_LT(mserve_utils::system_pressure_min_bar, mserve_utils::system_pressure_max_bar);
}

TEST(TestConfig, TemperatureLimits) {
  EXPECT_LT(mserve_utils::pt100_min, mserve_utils::pt100_max);
}

TEST(TestConfig, TimingLimits) {
  EXPECT_GE(mserve_utils::timing_min, 0.0);
  EXPECT_GT(mserve_utils::timing_max, mserve_utils::timing_min);
}

#include <gtest/gtest.h>
#include "mserve_utils/config.hpp"

TEST(TestConfig, DefaultName) {
  mserve_utils::Config cfg;
  EXPECT_EQ(cfg.name, "");
}

#include <gtest/gtest.h>
#include "mserve_drivechain/diff_drive.hpp"

using mserve_drivechain::DiffDrive;

// DDSM115 wheel geometry used in tests: separation=0.35m, radius=0.08m
static constexpr double SEP = 0.35;
static constexpr double RAD = 0.08;

TEST(DiffDrive, StraightForward) {
  DiffDrive dd(SEP, RAD);
  auto s = dd.compute(0.5, 0.0);
  double expected = 0.5 / RAD;
  EXPECT_NEAR(s.left,  expected, 1e-9);
  EXPECT_NEAR(s.right, expected, 1e-9);
}

TEST(DiffDrive, StraightReverse) {
  DiffDrive dd(SEP, RAD);
  auto s = dd.compute(-0.5, 0.0);
  double expected = -0.5 / RAD;
  EXPECT_NEAR(s.left,  expected, 1e-9);
  EXPECT_NEAR(s.right, expected, 1e-9);
}

TEST(DiffDrive, PivotTurnLeft) {
  DiffDrive dd(SEP, RAD);
  auto s = dd.compute(0.0, 1.0);
  double expected = (SEP / 2.0) / RAD;
  EXPECT_NEAR(s.left,  -expected, 1e-9);
  EXPECT_NEAR(s.right,  expected, 1e-9);
}

TEST(DiffDrive, PivotTurnRight) {
  DiffDrive dd(SEP, RAD);
  auto s = dd.compute(0.0, -1.0);
  double expected = (SEP / 2.0) / RAD;
  EXPECT_NEAR(s.left,   expected, 1e-9);
  EXPECT_NEAR(s.right, -expected, 1e-9);
}

TEST(DiffDrive, ArcTurnLeft) {
  DiffDrive dd(SEP, RAD);
  auto s = dd.compute(0.4, 0.5);
  EXPECT_NEAR(s.left,  (0.4 - 0.5 * SEP / 2.0) / RAD, 1e-9);
  EXPECT_NEAR(s.right, (0.4 + 0.5 * SEP / 2.0) / RAD, 1e-9);
  EXPECT_LT(s.left, s.right);  // left slower on left arc
}

TEST(DiffDrive, ZeroCommand) {
  DiffDrive dd(SEP, RAD);
  auto s = dd.compute(0.0, 0.0);
  EXPECT_DOUBLE_EQ(s.left,  0.0);
  EXPECT_DOUBLE_EQ(s.right, 0.0);
}

#pragma once

namespace mserve_base {

constexpr double kLinearSpeedMin     = 0.01;  // m/s
constexpr double kLinearSpeedMax     = 10.0;  // m/s
constexpr double kAngularSpeedMin    = 0.01;  // rad/s
constexpr double kAngularSpeedMax    = 10.0;  // rad/s

constexpr double kWheelSeparationMin = 0.05;  // m
constexpr double kWheelSeparationMax = 2.0;   // m
constexpr double kWheelRadiusMin     = 0.01;  // m
constexpr double kWheelRadiusMax     = 1.0;   // m
constexpr double kGearRatioMin       = 0.01;  // motor revs per wheel rev
constexpr double kGearRatioMax       = 100.0;

constexpr int kMotorIdMin = 1;
constexpr int kMotorIdMax = 253;

constexpr int kCmdVelTimeoutMin = 50;    // ms
constexpr int kCmdVelTimeoutMax = 5000;  // ms

constexpr double kFeedbackRateMin = 1.0;   // Hz
constexpr double kFeedbackRateMax = 50.0;  // Hz

constexpr int kMotorRpmMax = 200;   // matches mserve_drivechain's DDSM115 ceiling (±200 RPM)
constexpr int kMotorRpmMin = -200;

}  // namespace mserve_base

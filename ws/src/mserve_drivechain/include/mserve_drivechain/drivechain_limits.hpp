#pragma once

namespace mserve_drivechain {

constexpr int    kMaxRpm          = 200;   // DDSM115 speed loop ceiling (±200 RPM)
constexpr int    kMinRpm          = -200;
constexpr int    kMotorIdMin      = 1;
constexpr int    kMotorIdMax      = 253;
constexpr double kWheelSepMin     = 0.05;
constexpr double kWheelSepMax     = 2.0;
constexpr double kWheelRadiusMin  = 0.01;
constexpr double kWheelRadiusMax  = 0.5;
constexpr double kFeedbackRateMin = 1.0;
constexpr double kFeedbackRateMax = 50.0;
constexpr int    kCmdVelTimeoutMin = 50;
constexpr int    kCmdVelTimeoutMax = 5000;

}  // namespace mserve_drivechain

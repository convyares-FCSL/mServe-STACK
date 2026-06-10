#pragma once

namespace mserve_drivechain {

constexpr int kMaxRpm            = 200;   // DDSM115 speed loop ceiling (±200 RPM)
constexpr int kMinRpm            = -200;
constexpr int kMotorIdMin        = 1;
constexpr int kMotorIdMax        = 253;
constexpr int kMotorCountMin     = 1;
constexpr int kMotorCountMax     = 4;
constexpr int kCommandTimeoutMin = 50;
constexpr int kCommandTimeoutMax = 5000;
constexpr int kMotorAccelMin     = 0;     // DDSM115 'act' byte — 0 = instant
constexpr int kMotorAccelMax     = 255;
constexpr double kFeedbackRateMin = 1.0;
constexpr double kFeedbackRateMax = 50.0;

}  // namespace mserve_drivechain

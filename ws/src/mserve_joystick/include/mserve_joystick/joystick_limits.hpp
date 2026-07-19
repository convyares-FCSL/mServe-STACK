#pragma once

namespace mserve_joystick {

constexpr double kDeadzone = 0.1;

constexpr double kSpeedInitial = 0.4;  // m/s
constexpr double kSpeedMin     = 0.1;
constexpr double kSpeedMax     = 0.8;  // matches mserve_base limits.max_linear_speed default
constexpr double kSpeedStep    = 0.1;

constexpr double kAngularInitial = 0.6;  // rad/s
constexpr double kAngularMin     = 0.1;
constexpr double kAngularMax     = 1.2;  // matches mserve_base limits.max_angular_speed default
constexpr double kAngularStep    = 0.2;

}  // namespace mserve_joystick

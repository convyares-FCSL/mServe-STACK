#pragma once

namespace hyfleet_booster
{
    // Fixed boster parameters for VFD
    constexpr double speed_min = 0.0;
    constexpr double speed_max = 1600.0;

    // Fixed boster parameters for Proportional Control Soelnoid Valve (PCSV)
    constexpr double cpm_min = 4.0;
    constexpr double cpm_max = 18.0;

} // namespace hyfleet_booster
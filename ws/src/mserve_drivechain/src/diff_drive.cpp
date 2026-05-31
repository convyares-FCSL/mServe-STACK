#include "mserve_drivechain/diff_drive.hpp"

namespace mserve_drivechain {

// ==============================================================================
// DiffDrive implementation.
// ==============================================================================

DiffDrive::DiffDrive(double wheel_separation, double wheel_radius)
: half_sep_(wheel_separation / 2.0), wheel_radius_(wheel_radius)
{}

WheelSpeeds DiffDrive::compute(double linear_mps, double angular_rps) const
{
  return {
    (linear_mps - angular_rps * half_sep_) / wheel_radius_,
    (linear_mps + angular_rps * half_sep_) / wheel_radius_
  };
}

}  // namespace mserve_drivechain

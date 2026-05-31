#ifndef MSERVE_DRIVECHAIN_DIFF_DRIVE_HPP
#define MSERVE_DRIVECHAIN_DIFF_DRIVE_HPP

namespace mserve_drivechain {
 
  struct WheelSpeeds {
  double left;   // rad/s
  double right;  // rad/s
};

// ==============================================================================
// BaseNode: a lifecycle node that subscribes to cmd_vel, applies speed limits, and republishes safely.
// ==============================================================================

class DiffDrive {
public:
  DiffDrive(double wheel_separation, double wheel_radius);

  WheelSpeeds compute(double linear_mps, double angular_rps) const;

private:
  double half_sep_;
  double wheel_radius_;
};

}  // namespace mserve_drivechain

#endif  // MSERVE_DRIVECHAIN_DIFF_DRIVE_HPP

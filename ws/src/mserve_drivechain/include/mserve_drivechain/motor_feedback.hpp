#pragma once

namespace mserve_drivechain {

// Decoded feedback from one DDSM115 motor response packet.
struct MotorFeedback {
  int mode{2};        // operating mode: 1=current, 2=speed, 3=position
  int speed_rpm{0};   // signed, −200 to +200
  int torque{0};      // raw int16 torque value
  int position{0};    // 0–32767 maps to 0–360°
  int fault_code{0};  // 0 = healthy
  int temperature{0}; // °C, only valid after get_info
};

}  // namespace mserve_drivechain

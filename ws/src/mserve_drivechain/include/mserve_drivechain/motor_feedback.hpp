#pragma once

namespace mserve_drivechain {

// Motor feedback decoded from the ESP32 JSON response.
// Fields map to the JSON keys returned by the DDSM Driver HAT (A):
//   spd → speed_rpm,  err → fault_code,  crt → current,  tep → temperature
// position is not available in normal speed-loop feedback (stays 0).
struct MotorFeedback {
  int   mode{2};         // operating mode: 1=current, 2=speed, 3=position
  int   speed_rpm{0};    // signed, −200 to +200 (rpm)
  int   position{0};     // 0–32767 = 0–360° (not populated in speed-loop mode)
  int   fault_code{0};   // 0 = healthy (from "err" field)
  float current{0.0f};   // amps (from "crt" field)
  float temperature{0.0f}; // °C (from "tep" field)
};

}  // namespace mserve_drivechain

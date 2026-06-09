#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include "mserve_drivechain/motor_feedback.hpp"

namespace mserve_drivechain {

// Low-level DDSM115 protocol layer.
//
// In sim_mode the port is never opened; all commands succeed immediately
// and feedback echoes commanded speed. This lets the higher-level node
// and BT trees operate identically regardless of backend.
//
// All reads use select() so no call ever blocks longer than timeout_ms.
class DriveUart {
public:
  explicit DriveUart(bool sim_mode);
  ~DriveUart();

  DriveUart(const DriveUart &) = delete;
  DriveUart & operator=(const DriveUart &) = delete;

  // --- lifecycle ---
  bool open(const std::string & device, int baud = 115200);
  void close();
  bool is_open()   const;
  bool is_sim()    const { return sim_mode_; }

  // --- per-command operations ---

  // Send speed command and read back motor feedback.
  // rpm is clamped to [−200, +200] internally.
  bool set_speed(uint8_t id, int rpm, MotorFeedback & fb);

  // set_speed(id, 0) — convenience stop.
  bool stop(uint8_t id, MotorFeedback & fb);

  // Set operating mode. DDSM115: 1=current, 2=speed, 3=position.
  // Protocol quirk: mode byte goes in position [9], no CRC, no response.
  bool set_mode(uint8_t id, uint8_t mode);

  // Send a stop command to id and wait for its echo — verifies the motor is alive.
  bool ping(uint8_t id);

  // --- one-time setup (hardware only) ---

  // Broadcast id-check (0xC8). Only safe when exactly one motor is on the bus.
  // Returns the responding motor's ID, or -1 on timeout / CRC failure.
  int id_check();

  // Change motor ID. Sends the 5× change-id sequence then calls ping(new_id).
  bool change_id(uint8_t current_id, uint8_t new_id);

  // --- CRC utility (public so tests can use it) ---
  static uint8_t crc8(const uint8_t * data, size_t len);

private:
  bool write_packet(const uint8_t * pkt);
  bool read_bytes(uint8_t * buf, size_t n, int timeout_ms = 10);
  bool parse_feedback_115(const uint8_t * data, MotorFeedback & fb);

  bool sim_mode_;
  int  fd_ = -1;

  // Sim state: track commanded RPM per motor so feedback echoes it.
  std::unordered_map<uint8_t, int> sim_cmds_;
};

}  // namespace mserve_drivechain

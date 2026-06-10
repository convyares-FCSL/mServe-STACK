#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>

#include "drivechain_types.hpp"

namespace mserve_drivechain {

// DDSM Driver HAT (A) protocol layer — JSON over UART to onboard ESP32.
//
// The HAT's ESP32 accepts newline-terminated JSON commands and returns JSON
// feedback.  Raw DDSM115 binary packets are handled internally by the ESP32;
// this class never sends them directly.
//
// In sim_mode the port is never opened; all commands succeed immediately
// and feedback echoes commanded speed.
class DriveUart {
public:
  // The ESP32 sends a {"T":20020,"hb":N,"up":millis} liveness heartbeat
  // roughly every 1000 ms. board_alive() reports stale after this long
  // without a fresh one.
  static constexpr int kHeartbeatTimeoutMs = 3000;

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
  // accel is the DDSM115 'act' ramp byte (0-255); 0 = instant.
  bool set_speed(uint8_t id, int rpm, MotorFeedback & fb, uint8_t accel = 0);

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

  // True if the ESP32's liveness heartbeat ({"T":20020,...}) has been seen
  // within timeout_ms. Always true in sim_mode. Heartbeat lines are detected
  // transparently inside read_line(), so any UART traffic (set_speed,
  // set_mode, ...) keeps this fresh — no dedicated polling needed.
  bool board_alive(int timeout_ms = kHeartbeatTimeoutMs) const;

  // CRC-8/MAXIM — kept for unit-test compatibility (not used in JSON protocol).
  static uint8_t crc8(const uint8_t * data, size_t len);

private:
  // JSON protocol helpers
  bool send_json(const std::string & cmd);
  bool read_line(std::string & out, int timeout_ms);
  bool parse_json_feedback(const std::string & line, uint8_t expected_id, MotorFeedback & fb);

  bool sim_mode_;
  int  fd_ = -1;

  // Sim state: track commanded RPM per motor so feedback echoes it.
  std::unordered_map<uint8_t, int> sim_cmds_;

  // Last time a {"T":20020,...} heartbeat line was seen on the wire.
  std::chrono::steady_clock::time_point last_heartbeat_{};
};

}  // namespace mserve_drivechain

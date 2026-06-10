#include "include/drivechain_uart.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <thread>
#include <chrono>

namespace mserve_drivechain {

// ==============================================================================
// CRC-8/MAXIM — kept for unit-test compatibility, not used in JSON protocol
// ==============================================================================

uint8_t DriveUart::crc8(const uint8_t * data, size_t len)
{
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x01) crc = (crc >> 1) ^ 0x8C;
      else            crc >>= 1;
    }
  }
  return crc;
}

// ==============================================================================
// JSON helpers
// ==============================================================================

// Extract an integer field from a JSON string: {"key":value,...}
static int json_int(const std::string & s, const char * key, int def = 0)
{
  std::string k = std::string("\"") + key + "\":";
  auto pos = s.find(k);
  if (pos == std::string::npos) return def;
  try { return std::stoi(s.substr(pos + k.size())); }
  catch (...) { return def; }
}

static float json_float(const std::string & s, const char * key, float def = 0.0f)
{
  std::string k = std::string("\"") + key + "\":";
  auto pos = s.find(k);
  if (pos == std::string::npos) return def;
  try { return std::stof(s.substr(pos + k.size())); }
  catch (...) { return def; }
}

// {"T":20020,"hb":N,"up":millis} — periodic ESP32 liveness heartbeat.
static bool is_heartbeat_line(const std::string & s)
{
  return json_int(s, "T") == 20020;
}

bool DriveUart::send_json(const std::string & cmd)
{
  if (fd_ < 0) return false;
  std::string line = cmd + "\n";
  ssize_t n = write(fd_, line.c_str(), line.size());
  return n == static_cast<ssize_t>(line.size());
}

bool DriveUart::read_line(std::string & out, int timeout_ms)
{
  out.clear();
  if (fd_ < 0) return false;

  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (true) {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) return false;
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count();

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);
    timeval tv;
    tv.tv_sec  = us / 1'000'000;
    tv.tv_usec = us % 1'000'000;

    if (select(fd_ + 1, &fds, nullptr, nullptr, &tv) <= 0) return false;

    char c;
    if (read(fd_, &c, 1) <= 0) return false;
    if (c == '\n') {
      if (is_heartbeat_line(out)) {
        last_heartbeat_ = std::chrono::steady_clock::now();
        out.clear();
        continue;  // keep waiting for a real line until the deadline
      }
      return !out.empty();
    }
    if (c != '\r') out += c;  // skip CR in CRLF
  }
}

// Parse a JSON motor feedback line from the ESP32.
// Expected format: {"T":20010,"id":1,"typ":115,"spd":50,"crt":1.2,"act":3,"tep":35,"err":0}
bool DriveUart::parse_json_feedback(const std::string & line, uint8_t expected_id,
  MotorFeedback & fb)
{
  if (line.empty() || line[0] != '{') return false;
  const int t = json_int(line, "T");
  if (t != 20010 && t != 20030) return false;  // motor ctrl / info response

  const int id = json_int(line, "id", -1);
  if (id != static_cast<int>(expected_id)) return false;

  fb.mode        = 2;
  fb.speed_rpm   = json_int(line,   "spd");
  fb.fault_code  = json_int(line,   "err");
  fb.current     = json_float(line, "crt");
  fb.temperature = json_float(line, "tep");
  fb.position    = 0;  // not returned in speed-loop feedback
  return true;
}

// ==============================================================================
// Lifecycle
// ==============================================================================

DriveUart::DriveUart(bool sim_mode) : sim_mode_(sim_mode) {}

DriveUart::~DriveUart() { close(); }

bool DriveUart::open(const std::string & device, int baud)
{
  if (sim_mode_) return true;

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) return false;

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) { ::close(fd_); fd_ = -1; return false; }

  speed_t speed = B115200;
  if (baud == 9600)   speed = B9600;
  if (baud == 57600)  speed = B57600;
  if (baud == 115200) speed = B115200;

  // In Python-3-style termios, ispeed=tty[4] ospeed=tty[5], but in C termios
  // we use cfsetspeed.
  cfsetspeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK |
                   INPCK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) { ::close(fd_); fd_ = -1; return false; }

  tcflush(fd_, TCIOFLUSH);
  return true;
}

void DriveUart::close()
{
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  sim_cmds_.clear();
  last_heartbeat_ = {};
}

bool DriveUart::is_open() const
{
  return sim_mode_ || (fd_ >= 0);
}

bool DriveUart::board_alive(int timeout_ms) const
{
  if (sim_mode_) return true;
  if (last_heartbeat_ == std::chrono::steady_clock::time_point{}) return false;
  return std::chrono::steady_clock::now() - last_heartbeat_ < std::chrono::milliseconds(timeout_ms);
}

// ==============================================================================
// Motor commands
// ==============================================================================

bool DriveUart::set_speed(uint8_t id, int rpm, MotorFeedback & fb)
{
  rpm = std::clamp(rpm, -200, 200);

  if (sim_mode_) {
    sim_cmds_[id] = rpm;
    fb.mode        = 2;
    fb.speed_rpm   = rpm;
    fb.fault_code  = 0;
    fb.current     = 0.0f;
    fb.temperature = 0.0f;
    return true;
  }

  char buf[64];
  std::snprintf(buf, sizeof(buf),
    "{\"T\":10010,\"id\":%d,\"cmd\":%d,\"act\":0}", id, rpm);

  if (!send_json(buf)) return false;

  // The ESP32 may send unrelated lines first; scan up to 5 lines for a match.
  for (int i = 0; i < 5; ++i) {
    std::string line;
    if (!read_line(line, 100)) return false;
    if (parse_json_feedback(line, id, fb)) return true;
  }
  return false;
}

bool DriveUart::stop(uint8_t id, MotorFeedback & fb)
{
  return set_speed(id, 0, fb);
}

bool DriveUart::set_mode(uint8_t id, uint8_t mode)
{
  if (sim_mode_) return true;

  char buf[64];
  std::snprintf(buf, sizeof(buf),
    "{\"T\":10012,\"id\":%d,\"mode\":%d}", id, static_cast<int>(mode));

  if (!send_json(buf)) return false;

  // Mode changes may or may not return feedback — drain any response.
  std::string line;
  read_line(line, 150);
  return true;
}

bool DriveUart::ping(uint8_t id)
{
  if (sim_mode_) return true;
  MotorFeedback fb;
  return set_speed(id, 0, fb) && fb.fault_code == 0;
}

// ==============================================================================
// One-time setup
// ==============================================================================

int DriveUart::id_check()
{
  if (sim_mode_) return 1;

  // Only safe when exactly one motor is on the bus.
  if (!send_json("{\"T\":10031}")) return -1;

  std::string line;
  if (!read_line(line, 500)) return -1;

  return json_int(line, "id", -1);
}

bool DriveUart::change_id(uint8_t current_id, uint8_t new_id)
{
  if (sim_mode_) return true;
  (void)current_id;  // single motor on bus — no need to address by current ID

  char buf[64];
  std::snprintf(buf, sizeof(buf), "{\"T\":10011,\"id\":%d}", static_cast<int>(new_id));

  if (!send_json(buf)) return false;

  // Give the ESP32 time to reprogram the motor ID.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  return ping(new_id);
}

}  // namespace mserve_drivechain

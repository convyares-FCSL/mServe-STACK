#include "include/drivechain_uart.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <thread>
#include <chrono>

namespace mserve_drivechain {

// ==============================================================================
// CRC-8/MAXIM (Dallas/Maxim 1-Wire CRC)
// Polynomial 0x8C (reflected). Initial value 0x00.
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
// Construction / lifecycle
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

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | INPCK | ISTRIP | INLCR | IGNCR | ICRNL);
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
}

bool DriveUart::is_open() const
{
  return sim_mode_ || (fd_ >= 0);
}

// ==============================================================================
// Internal helpers
// ==============================================================================

bool DriveUart::write_packet(const uint8_t * pkt)
{
  if (sim_mode_) return true;
  if (fd_ < 0) return false;
  ssize_t written = write(fd_, pkt, 10);
  return written == 10;
}

bool DriveUart::read_bytes(uint8_t * buf, size_t n, int timeout_ms)
{
  if (sim_mode_) return false;  // caller handles sim path separately
  if (fd_ < 0) return false;

  size_t received = 0;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (received < n) {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) return false;

    auto remaining_us = std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count();

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);
    timeval tv;
    tv.tv_sec  = remaining_us / 1'000'000;
    tv.tv_usec = remaining_us % 1'000'000;

    int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (ret <= 0) return false;

    ssize_t got = read(fd_, buf + received, n - received);
    if (got <= 0) return false;
    received += static_cast<size_t>(got);
  }
  return true;
}

bool DriveUart::parse_feedback_115(const uint8_t * data, MotorFeedback & fb)
{
  // Verify CRC over bytes [0..8]
  if (crc8(data, 9) != data[9]) return false;

  fb.mode  = data[1];

  fb.torque = static_cast<int16_t>((data[2] << 8) | data[3]);

  fb.speed_rpm = static_cast<int16_t>((data[4] << 8) | data[5]);

  // position field (normal feedback; not get_info)
  fb.position  = (data[6] << 8) | data[7];
  fb.fault_code = data[8];
  return true;
}

// ==============================================================================
// Motor commands
// ==============================================================================

bool DriveUart::set_speed(uint8_t id, int rpm, MotorFeedback & fb)
{
  rpm = std::clamp(rpm, -200, 200);

  if (sim_mode_) {
    sim_cmds_[id] = rpm;
    fb.mode       = 2;
    fb.speed_rpm  = rpm;
    fb.fault_code = 0;
    return true;
  }

  uint8_t pkt[10] = {};
  int16_t cmd16 = static_cast<int16_t>(rpm);
  pkt[0] = id;
  pkt[1] = 0x64;
  pkt[2] = (cmd16 >> 8) & 0xFF;
  pkt[3] = cmd16 & 0xFF;
  // bytes 4-8 remain 0x00
  pkt[9] = crc8(pkt, 9);

  if (!write_packet(pkt)) return false;

  uint8_t resp[10];
  if (!read_bytes(resp, 10, 10)) return false;
  if (resp[0] != id) return false;

  return parse_feedback_115(resp, fb);
}

bool DriveUart::stop(uint8_t id, MotorFeedback & fb)
{
  return set_speed(id, 0, fb);
}

bool DriveUart::set_mode(uint8_t id, uint8_t mode)
{
  if (sim_mode_) return true;

  uint8_t pkt[10] = {};
  pkt[0] = id;
  pkt[1] = 0xA0;
  // bytes 2-8 remain 0x00
  pkt[9] = mode;  // DDSM115 protocol: mode in byte[9], not CRC

  return write_packet(pkt);
  // No response for mode change
}

bool DriveUart::ping(uint8_t id)
{
  if (sim_mode_) return true;

  MotorFeedback fb;
  return stop(id, fb) && fb.fault_code == 0;
}

// ==============================================================================
// One-time setup
// ==============================================================================

int DriveUart::id_check()
{
  if (sim_mode_) return 1;

  // Broadcast packet — only use when exactly one motor is on the bus.
  uint8_t pkt[10] = {0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE};
  if (!write_packet(pkt)) return -1;

  uint8_t resp[10];
  if (!read_bytes(resp, 10, 20)) return -1;

  if (crc8(resp, 9) != resp[9]) return -1;

  return static_cast<int>(resp[0]);
}

bool DriveUart::change_id(uint8_t current_id, uint8_t new_id)
{
  if (sim_mode_) return true;

  uint8_t pkt[10] = {};
  pkt[0] = 0xAA;
  pkt[1] = 0x55;
  pkt[2] = 0x53;
  pkt[3] = new_id;
  // bytes 4-8 remain 0x00
  pkt[9] = crc8(pkt, 9);

  for (int i = 0; i < 5; ++i) {
    write_packet(pkt);
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }

  (void)current_id;
  return ping(new_id);
}

}  // namespace mserve_drivechain

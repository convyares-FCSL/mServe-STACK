#include "mserve_sensehat/led_matrix.hpp"

#include <cstdio>
#include <fstream>

#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

namespace mserve_sensehat {

std::string LedMatrix::resolveDevice(const std::string & fallback)
{
  DIR * dir = ::opendir("/sys/class/graphics");
  if (dir == nullptr) {
    return fallback;
  }
  std::string found;
  while (dirent * entry = ::readdir(dir)) {
    std::string fb_name = entry->d_name;
    if (fb_name.rfind("fb", 0) != 0) {
      continue;
    }
    std::ifstream name_file("/sys/class/graphics/" + fb_name + "/name");
    std::string driver;
    if (name_file && std::getline(name_file, driver) && driver == "RPi-Sense FB") {
      found = "/dev/" + fb_name;
      break;
    }
  }
  ::closedir(dir);
  return found.empty() ? fallback : found;
}

bool LedMatrix::open()
{
  std::string device = resolveDevice();
  fd_ = ::open(device.c_str(), O_RDWR);
  if (fd_ < 0) {
    std::fprintf(stderr, "[mserve_sensehat] failed to open LED matrix at %s\n", device.c_str());
    return false;
  }
  std::fprintf(stderr, "[mserve_sensehat] LED matrix: %s\n", device.c_str());
  return true;
}

void LedMatrix::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

uint16_t LedMatrix::rgb565(uint8_t r, uint8_t g, uint8_t b)
{
  uint16_t r5 = (r >> 3) & 0x1F;
  uint16_t g6 = (g >> 2) & 0x3F;
  uint16_t b5 = (b >> 3) & 0x1F;
  return static_cast<uint16_t>((r5 << 11) | (g6 << 5) | b5);
}

void LedMatrix::clear(uint16_t rgb565_color)
{
  back_buffer_.fill(rgb565_color);
}

void LedMatrix::drawBitmap(const std::array<uint8_t, kSize> & bits, uint16_t fg, uint16_t bg)
{
  for (int row = 0; row < kSize; ++row) {
    for (int col = 0; col < kSize; ++col) {
      bool lit = (bits[row] & (1 << (kSize - 1 - col))) != 0;
      back_buffer_[static_cast<size_t>(row) * kSize + col] = lit ? fg : bg;
    }
  }
}

void LedMatrix::present()
{
  if (fd_ < 0) {
    return;
  }
  // Every present() after the first was silently landing past the end of
  // this 128-byte device (8x8x2 bytes, no padding) and failing: write()
  // advances the fd's offset, so without seeking back to 0 first, the
  // second call starts writing at byte 128 — one past the device's entire
  // size. The very first present() (the boot-time X icon) always "worked"
  // purely because a freshly-opened fd starts at offset 0; every redraw
  // after that (e.g. X -> O on connect) silently did nothing, and the
  // ignored write() return value hid the failure. Confirmed on real
  // hardware: the icon never changed after the initial draw until this fix.
  lseek(fd_, 0, SEEK_SET);
  ssize_t want = static_cast<ssize_t>(back_buffer_.size() * sizeof(uint16_t));
  ssize_t got = write(fd_, back_buffer_.data(), want);
  (void)got;  // best-effort — a dropped LED refresh isn't worth crashing the node over
}

}  // namespace mserve_sensehat

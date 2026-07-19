#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace mserve_sensehat {

// Direct /dev/fbN access for the Sense HAT's 8x8 RGB565 LED matrix (kernel
// driver name "RPi-Sense FB"). Deliberately not shared with
// mserve_display::Framebuffer — that class's mmap/back-buffer/text-
// rendering machinery is built for a 480x320 SPI panel; this is an 8x8
// fixed-size grid with two icons, and forcing a shared abstraction across
// such different scales would cost more than the ~30 lines it'd save.
class LedMatrix
{
public:
  static constexpr int kSize = 8;

  bool open();
  void close();
  bool isOpen() const {return fd_ >= 0;}

  void clear(uint16_t rgb565 = 0x0000);
  // bits: 8 rows, MSB = leftmost column (same convention as
  // mserve_display's font8x8.hpp), 1 = draw fg, 0 = leave background.
  void drawBitmap(const std::array<uint8_t, kSize> & bits, uint16_t fg, uint16_t bg = 0x0000);
  void present();

  static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b);

  // Looks up /sys/class/graphics/fb*/name for "RPi-Sense FB" — framebuffer
  // *numbering* is probe-order-dependent, not fixed (see
  // mserve_display::resolveFramebufferDevice's comment for the exact
  // incident that motivated checking by name instead of assuming a path).
  static std::string resolveDevice(const std::string & fallback = "/dev/fb0");

private:
  int fd_ = -1;
  std::array<uint16_t, kSize * kSize> back_buffer_{};
};

}  // namespace mserve_sensehat

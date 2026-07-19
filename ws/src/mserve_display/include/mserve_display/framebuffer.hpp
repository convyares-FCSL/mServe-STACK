#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mserve_display {

// Direct /dev/fb0 access (RGB565, confirmed by a live color test against
// the physical ELEGOO screen — see transfer.md). Draws into an in-process
// back buffer; present() flushes the whole frame in one operation. This
// SPI-attached panel almost certainly has no FBIOPAN_DISPLAY page-flipping
// support (real double buffering isn't available), so present() is a
// single mmap'd memcpy (or, if the driver doesn't support mmap, a single
// bulk write() — probed once at open() and never mixed per-call). Callers
// must never call present() per-primitive: a full-frame SPI flush is slow
// (100ms+), so draw everything for one logical redraw into the back
// buffer first, then present() once.
// Looks up /sys/class/graphics/fb*/name for a framebuffer whose driver name
// matches driver_name, returning "/dev/fbN". Framebuffer *numbering* is
// probe-order-dependent, not fixed — it shifted on this robot (the ELEGOO
// panel's fb_ili9486 driver went from fb0 to fb1) the moment a Pi Sense
// HAT's own rpisense_fb driver was added ahead of it, silently breaking a
// hardcoded device path with no error (writes just went to the wrong
// device). Falls back to `fallback` if no match is found.
std::string resolveFramebufferDevice(
  const std::string & driver_name = "fb_ili9486",
  const std::string & fallback = "/dev/fb0");

class Framebuffer
{
public:
  // flip_180: mirrors every present() flush (both axes) before it hits the
  // physical panel. The dtoverlay's rotate=90 param and the physical
  // panel's actual mount orientation are two independent things — a live
  // hardware test showed framebuffer content that was verified correct in
  // memory (via a raw /dev/fb0 dump) still rendering upside down on the
  // panel. A Y-only mirror was tried first (on the theory that a separate
  // cmd_vel test already showed the eyes' left/right was correct) but left
  // the menu screen still upside down — this hardware genuinely needs the
  // full rotation. The Face screen's left/right issue turned out to be an
  // unrelated bug in the eye-direction sign, not this transform — see
  // DisplayNode::onCmdVel.
  explicit Framebuffer(std::string device = "/dev/fb0", bool flip_180 = false);
  ~Framebuffer();
  Framebuffer(const Framebuffer &) = delete;
  Framebuffer & operator=(const Framebuffer &) = delete;

  // Returns false (and logs via stderr — this runs before any rclcpp
  // logger is necessarily available, callers should also log via
  // RCLCPP_ERROR on failure) if the device can't be opened at all. A
  // failed mmap() specifically is not fatal — present() transparently
  // falls back to write().
  bool open();
  void close();
  bool isOpen() const {return fd_ >= 0;}

  int width() const {return width_;}
  int height() const {return height_;}

  void clear(uint16_t rgb565 = 0x0000);
  void setPixel(int x, int y, uint16_t rgb565);
  void fillRect(int x, int y, int w, int h, uint16_t rgb565);
  void drawRect(int x, int y, int w, int h, uint16_t rgb565);
  void fillCircle(int cx, int cy, int r, uint16_t rgb565);
  void drawCircle(int cx, int cy, int r, uint16_t rgb565);

  // Renders in caps regardless of input case — see font8x8.hpp.
  void drawChar(int x, int y, char c, uint16_t fg, int scale = 1);
  void drawText(int x, int y, const std::string & text, uint16_t fg, int scale = 1);
  int textWidth(const std::string & text, int scale = 1) const;
  static constexpr int textHeight(int scale = 1) {return 7 * scale;}

  // Flushes the back buffer to the physical display. Cheap to call
  // whenever a logical redraw is needed — expensive to call per-primitive.
  void present();

  static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b);

private:
  std::string device_;
  bool flip_180_ = false;
  int fd_ = -1;
  uint16_t * mmap_base_ = nullptr;
  size_t mmap_len_ = 0;
  bool use_mmap_ = false;

  std::vector<uint16_t> back_buffer_;
  int width_ = 0;
  int height_ = 0;
  int stride_px_ = 0;  // may exceed width_ — padded scanlines are common on fbdev drivers
};

}  // namespace mserve_display

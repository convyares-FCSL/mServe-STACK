#include "mserve_display/framebuffer.hpp"
#include "mserve_display/font8x8.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <dirent.h>

#include <fstream>

namespace mserve_display {

std::string resolveFramebufferDevice(const std::string & driver_name, const std::string & fallback)
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
    if (name_file && std::getline(name_file, driver) && driver == driver_name) {
      found = "/dev/" + fb_name;
      break;
    }
  }
  ::closedir(dir);
  return found.empty() ? fallback : found;
}

Framebuffer::Framebuffer(std::string device, bool flip_180)
: device_(std::move(device)), flip_180_(flip_180) {}

Framebuffer::~Framebuffer() {close();}

bool Framebuffer::open()
{
  fd_ = ::open(device_.c_str(), O_RDWR);
  if (fd_ < 0) {
    std::fprintf(stderr, "[mserve_display] failed to open %s\n", device_.c_str());
    return false;
  }

  fb_var_screeninfo vinfo{};
  fb_fix_screeninfo finfo{};
  if (ioctl(fd_, FBIOGET_VSCREENINFO, &vinfo) < 0 ||
    ioctl(fd_, FBIOGET_FSCREENINFO, &finfo) < 0)
  {
    std::fprintf(stderr, "[mserve_display] FBIOGET_*SCREENINFO failed on %s\n", device_.c_str());
    close();
    return false;
  }

  width_ = static_cast<int>(vinfo.xres);
  height_ = static_cast<int>(vinfo.yres);
  int bytes_per_pixel = static_cast<int>(vinfo.bits_per_pixel) / 8;
  stride_px_ = bytes_per_pixel > 0 ?
    static_cast<int>(finfo.line_length) / bytes_per_pixel : width_;
  if (stride_px_ <= 0) {
    stride_px_ = width_;
  }

  back_buffer_.assign(static_cast<size_t>(width_) * height_, 0x0000);

  mmap_len_ = finfo.smem_len > 0 ?
    finfo.smem_len : static_cast<size_t>(stride_px_) * height_ * sizeof(uint16_t);
  void * m = mmap(nullptr, mmap_len_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  if (m == MAP_FAILED) {
    std::fprintf(
      stderr,
      "[mserve_display] mmap failed on %s, falling back to write()\n", device_.c_str());
    mmap_base_ = nullptr;
    use_mmap_ = false;
  } else {
    mmap_base_ = static_cast<uint16_t *>(m);
    use_mmap_ = true;
  }

  std::fprintf(
    stderr, "[mserve_display] %s: %dx%d, stride=%dpx, mmap=%s\n",
    device_.c_str(), width_, height_, stride_px_, use_mmap_ ? "yes" : "no (write() fallback)");
  return true;
}

void Framebuffer::close()
{
  if (mmap_base_ != nullptr) {
    munmap(mmap_base_, mmap_len_);
    mmap_base_ = nullptr;
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  use_mmap_ = false;
}

void Framebuffer::present()
{
  if (fd_ < 0 || back_buffer_.empty()) {
    return;
  }

  // flip_180_: on this hardware the panel needs a full 180-degree rotation
  // to render right-side up (confirmed live — a Y-only mirror, tried first,
  // left the menu screen still upside down). Source is (W*H-1-i), i.e. the
  // whole buffer reversed. Left/right on the Face screen turned out to be a
  // separate bug, unrelated to this transform — see DisplayNode::onCmdVel.
  if (use_mmap_ && mmap_base_ != nullptr) {
    for (int y = 0; y < height_; ++y) {
      uint16_t * dst = mmap_base_ + static_cast<size_t>(y) * stride_px_;
      if (flip_180_) {
        const uint16_t * src = back_buffer_.data() + static_cast<size_t>(height_ - 1 - y) * width_;
        for (int x = 0; x < width_; ++x) {
          dst[x] = src[width_ - 1 - x];
        }
      } else {
        std::memcpy(
          dst, back_buffer_.data() + static_cast<size_t>(y) * width_,
          static_cast<size_t>(width_) * sizeof(uint16_t));
      }
    }
    return;
  }

  // write() fallback — one row at a time so a padded stride on the device
  // side doesn't corrupt adjacent rows.
  std::vector<uint16_t> flipped_row;
  if (flip_180_) {
    flipped_row.resize(static_cast<size_t>(width_));
  }
  for (int y = 0; y < height_; ++y) {
    off_t offset = static_cast<off_t>(y) * stride_px_ * sizeof(uint16_t);
    lseek(fd_, offset, SEEK_SET);
    const uint16_t * row;
    if (flip_180_) {
      const uint16_t * src = back_buffer_.data() + static_cast<size_t>(height_ - 1 - y) * width_;
      for (int x = 0; x < width_; ++x) {
        flipped_row[x] = src[width_ - 1 - x];
      }
      row = flipped_row.data();
    } else {
      row = back_buffer_.data() + static_cast<size_t>(y) * width_;
    }
    ssize_t want = static_cast<ssize_t>(width_) * sizeof(uint16_t);
    ssize_t got = write(fd_, row, want);
    (void)got;  // best-effort — a display glitch isn't worth crashing the node over
  }
}

uint16_t Framebuffer::rgb565(uint8_t r, uint8_t g, uint8_t b)
{
  uint16_t r5 = (r >> 3) & 0x1F;
  uint16_t g6 = (g >> 2) & 0x3F;
  uint16_t b5 = (b >> 3) & 0x1F;
  return static_cast<uint16_t>((r5 << 11) | (g6 << 5) | b5);
}

void Framebuffer::clear(uint16_t rgb565_color)
{
  std::fill(back_buffer_.begin(), back_buffer_.end(), rgb565_color);
}

void Framebuffer::setPixel(int x, int y, uint16_t rgb565_color)
{
  if (x < 0 || x >= width_ || y < 0 || y >= height_) {
    return;
  }
  back_buffer_[static_cast<size_t>(y) * width_ + x] = rgb565_color;
}

void Framebuffer::fillRect(int x, int y, int w, int h, uint16_t rgb565_color)
{
  int x0 = std::max(x, 0);
  int y0 = std::max(y, 0);
  int x1 = std::min(x + w, width_);
  int y1 = std::min(y + h, height_);
  for (int row = y0; row < y1; ++row) {
    uint16_t * dst = back_buffer_.data() + static_cast<size_t>(row) * width_;
    for (int col = x0; col < x1; ++col) {
      dst[col] = rgb565_color;
    }
  }
}

void Framebuffer::drawRect(int x, int y, int w, int h, uint16_t rgb565_color)
{
  fillRect(x, y, w, 1, rgb565_color);
  fillRect(x, y + h - 1, w, 1, rgb565_color);
  fillRect(x, y, 1, h, rgb565_color);
  fillRect(x + w - 1, y, 1, h, rgb565_color);
}

void Framebuffer::fillCircle(int cx, int cy, int r, uint16_t rgb565_color)
{
  // Midpoint circle, filled via horizontal spans per octant pair.
  int x = r, y = 0, err = 0;
  while (x >= y) {
    fillRect(cx - x, cy + y, 2 * x + 1, 1, rgb565_color);
    fillRect(cx - x, cy - y, 2 * x + 1, 1, rgb565_color);
    fillRect(cx - y, cy + x, 2 * y + 1, 1, rgb565_color);
    fillRect(cx - y, cy - x, 2 * y + 1, 1, rgb565_color);
    y += 1;
    err += 1 + 2 * y;
    if (2 * (err - x) + 1 > 0) {
      x -= 1;
      err += 1 - 2 * x;
    }
  }
}

void Framebuffer::drawCircle(int cx, int cy, int r, uint16_t rgb565_color)
{
  int x = r, y = 0, err = 0;
  while (x >= y) {
    setPixel(cx + x, cy + y, rgb565_color);
    setPixel(cx + y, cy + x, rgb565_color);
    setPixel(cx - y, cy + x, rgb565_color);
    setPixel(cx - x, cy + y, rgb565_color);
    setPixel(cx - x, cy - y, rgb565_color);
    setPixel(cx - y, cy - x, rgb565_color);
    setPixel(cx + y, cy - x, rgb565_color);
    setPixel(cx + x, cy - y, rgb565_color);
    y += 1;
    err += 1 + 2 * y;
    if (2 * (err - x) + 1 > 0) {
      x -= 1;
      err += 1 - 2 * x;
    }
  }
}

void Framebuffer::drawChar(int x, int y, char c, uint16_t fg, int scale)
{
  const Glyph & glyph = glyphFor(c);
  for (int row = 0; row < kFontRows; ++row) {
    uint8_t bits = glyph[row];
    for (int col = 0; col < kFontCols; ++col) {
      if ((bits & (1 << (kFontCols - 1 - col))) == 0) {
        continue;
      }
      fillRect(x + col * scale, y + row * scale, scale, scale, fg);
    }
  }
}

void Framebuffer::drawText(int x, int y, const std::string & text, uint16_t fg, int scale)
{
  int cursor = x;
  int advance = (kFontCols + 1) * scale;
  for (char c : text) {
    drawChar(cursor, y, c, fg, scale);
    cursor += advance;
  }
}

int Framebuffer::textWidth(const std::string & text, int scale) const
{
  if (text.empty()) {
    return 0;
  }
  int advance = (kFontCols + 1) * scale;
  return static_cast<int>(text.size()) * advance - scale;  // no trailing gap
}

}  // namespace mserve_display

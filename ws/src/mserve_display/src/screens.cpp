#include "mserve_display/screens.hpp"

#include <algorithm>
#include <cstdio>

namespace mserve_display {

namespace
{
// Small fixed palette — RGB565.
uint16_t colBg() {return Framebuffer::rgb565(10, 15, 30);}
uint16_t colInfoBg() {return Framebuffer::rgb565(0, 0, 0);}
uint16_t colWhite() {return Framebuffer::rgb565(255, 255, 255);}
uint16_t colPupil() {return Framebuffer::rgb565(10, 10, 10);}
uint16_t colButton() {return Framebuffer::rgb565(30, 90, 160);}
uint16_t colButtonBorder() {return Framebuffer::rgb565(120, 190, 255);}
uint16_t colOk() {return Framebuffer::rgb565(80, 220, 100);}
uint16_t colFail() {return Framebuffer::rgb565(230, 70, 70);}
uint16_t colMuted() {return Framebuffer::rgb565(150, 160, 180);}

void centeredText(
  Framebuffer & fb, const Rect & r, const std::string & text, uint16_t fg, int scale)
{
  int tw = fb.textWidth(text, scale);
  int th = Framebuffer::textHeight(scale);
  int x = r.x + (r.w - tw) / 2;
  int y = r.y + (r.h - th) / 2;
  fb.drawText(x, y, text, fg, scale);
}
}  // namespace

std::array<Rect, 3> menuButtonRects(int screen_w, int /*screen_h*/)
{
  const int button_w = 300;
  const int button_h = 70;
  const int x = (screen_w - button_w) / 2;
  return {
    Rect{x, 20, button_w, button_h},   // Connect
    Rect{x, 115, button_w, button_h},  // Info
    Rect{x, 210, button_w, button_h},  // Face
  };
}

MenuButton hitTestMenu(int x, int y, const std::array<Rect, 3> & r)
{
  if (r[0].contains(x, y)) {return MenuButton::Connect;}
  if (r[1].contains(x, y)) {return MenuButton::Info;}
  if (r[2].contains(x, y)) {return MenuButton::Face;}
  return MenuButton::None;
}

void renderFace(Framebuffer & fb, const DisplayState & state)
{
  fb.clear(colBg());

  const int cy = 160;
  const int left_cx = 160;
  const int right_cx = 320;
  const int sclera_r = 55;
  const int pupil_r = 24;
  const int max_offset = 28;

  int offset = static_cast<int>(state.eye_direction * max_offset);

  fb.fillCircle(left_cx, cy, sclera_r, colWhite());
  fb.fillCircle(left_cx + offset, cy, pupil_r, colPupil());
  fb.fillCircle(right_cx, cy, sclera_r, colWhite());
  fb.fillCircle(right_cx + offset, cy, pupil_r, colPupil());
}

void renderMenu(Framebuffer & fb, const DisplayState & state, const std::array<Rect, 3> & r)
{
  fb.clear(colBg());

  static const std::array<std::string, 3> kLabels = {"CONNECT", "INFO", "FACE"};
  for (size_t i = 0; i < r.size(); ++i) {
    fb.fillRect(r[i].x, r[i].y, r[i].w, r[i].h, colButton());
    fb.drawRect(r[i].x, r[i].y, r[i].w, r[i].h, colButtonBorder());
    centeredText(fb, r[i], kLabels[i], colWhite(), 3);
  }

  Rect status_area{r[0].x, r[2].y + r[2].h + 15, r[0].w, 30};
  if (state.connect_in_flight) {
    centeredText(fb, status_area, "CONNECTING...", colMuted(), 2);
  } else if (state.connect_result_valid) {
    uint16_t color = state.connect_result_success ? colOk() : colFail();
    std::string text = state.connect_result_success ? "CONNECTED" : "CONNECT FAILED";
    centeredText(fb, status_area, text, color, 2);
  }
}

void renderInfo(Framebuffer & fb, const DisplayState & state)
{
  fb.clear(colInfoBg());

  const int scale = 2;
  const int line_h = Framebuffer::textHeight(scale) + 6;
  int y = 10;
  const int x = 10;

  char buf[96];

  fb.drawText(x, y, "IP: " + state.ip_address, colWhite(), scale);
  y += line_h;

  std::snprintf(
    buf, sizeof(buf), "DRIVE: %s", state.drivechain_state.c_str());
  fb.drawText(x, y, buf, colWhite(), scale);
  y += line_h;

  std::snprintf(buf, sizeof(buf), "BASE: %s", state.base_state.c_str());
  fb.drawText(x, y, buf, colWhite(), scale);
  y += line_h;

  std::snprintf(
    buf, sizeof(buf), "BATT: %.0f%% BOARD: %s",
    static_cast<double>(state.drivechain_status.battery_level),
    state.drivechain_status.board_alive ? "OK" : "DOWN");
  fb.drawText(x, y, buf, colWhite(), scale);
  y += line_h + 6;

  for (const auto & motor : state.motor_feedback.motors) {
    std::snprintf(
      buf, sizeof(buf), "%s: %.0fRPM %.1fC F%u",
      motor.name.c_str(), static_cast<double>(motor.velocity_rpm),
      static_cast<double>(motor.temperature_c), static_cast<unsigned>(motor.fault_code));
    fb.drawText(x, y, buf, colWhite(), scale);
    y += line_h;
  }
}

}  // namespace mserve_display

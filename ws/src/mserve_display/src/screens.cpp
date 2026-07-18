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
// Pure black, not colPupil()'s (10,10,10) — the brows sit directly on
// colBg() (10,15,30), and those two are close enough in RGB565 that they
// were nearly indistinguishable on the physical panel (confirmed
// invisible in a live hardware test) despite showing up fine in a raw
// framebuffer dump viewed at full size on a normal monitor. Pupils don't
// have this problem — they sit on the white sclera, not the background.
uint16_t colBrow() {return Framebuffer::rgb565(0, 0, 0);}
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

// Rasterized as vertical strips per column (same technique as
// Framebuffer::fillCircle uses horizontal spans per row) — a straight bar
// that slants by `tilt` px across its width, no new Framebuffer primitive
// needed. Pupil position alone is too subtle to read gaze direction from at
// a glance (two symmetric circles) — the brow slants the same way the eyes
// shift, giving a much more obvious visual cue.
void drawBrow(Framebuffer & fb, int cx, int y, int half_w, int thickness, int tilt, uint16_t color)
{
  int x0 = cx - half_w;
  int x1 = cx + half_w;
  for (int x = x0; x <= x1; ++x) {
    double frac = half_w > 0 ? static_cast<double>(x - cx) / half_w : 0.0;
    int dy = static_cast<int>(frac * tilt);
    fb.fillRect(x, y + dy - thickness / 2, 1, thickness, color);
  }
}
}  // namespace

std::array<Rect, 4> menuButtonRects(int screen_w, int /*screen_h*/)
{
  const int button_w = 300;
  const int button_h = 55;
  const int x = (screen_w - button_w) / 2;
  return {
    Rect{x, 10, button_w, button_h},   // Connect
    Rect{x, 78, button_w, button_h},   // Info
    Rect{x, 146, button_w, button_h},  // Face
    Rect{x, 214, button_w, button_h},  // Calibrate
  };
}

MenuButton hitTestMenu(int x, int y, const std::array<Rect, 4> & r)
{
  if (r[0].contains(x, y)) {return MenuButton::Connect;}
  if (r[1].contains(x, y)) {return MenuButton::Info;}
  if (r[2].contains(x, y)) {return MenuButton::Face;}
  if (r[3].contains(x, y)) {return MenuButton::Calibrate;}
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

  // Negated: eye_direction follows REP-103 (positive angular.z = turn
  // left), but left_cx/right_cx increase left-to-right in screen-pixel
  // space, so a positive (unnegated) offset shifted pupils toward higher x
  // — visually rightward — for a "turn left" command. Confirmed backwards
  // via real web UI driving (btn-left sends +angular.z, eyes visibly went
  // right) even with the framebuffer's own 180-degree flip_180 correction
  // already applied and independently confirmed correct on the Menu
  // screen — this is a separate bug in the offset math itself, not the
  // physical panel orientation.
  int offset = static_cast<int>(-state.eye_direction * max_offset);

  fb.fillCircle(left_cx, cy, sclera_r, colWhite());
  fb.fillCircle(left_cx + offset, cy, pupil_r, colPupil());
  fb.fillCircle(right_cx, cy, sclera_r, colWhite());
  fb.fillCircle(right_cx + offset, cy, pupil_r, colPupil());

  const int brow_half_w = 40;
  const int brow_thickness = 8;
  const int brow_y = cy - sclera_r - 18;
  const int brow_tilt = offset;  // same signed shift the pupils use — brow slants the same way
  drawBrow(fb, left_cx + offset, brow_y, brow_half_w, brow_thickness, brow_tilt, colBrow());
  drawBrow(fb, right_cx + offset, brow_y, brow_half_w, brow_thickness, brow_tilt, colBrow());
}

void renderMenu(Framebuffer & fb, const DisplayState & state, const std::array<Rect, 4> & r)
{
  fb.clear(colBg());

  // Connect's label is a status indicator, not a real toggle — there is no
  // disconnect capability in mserve_drivechain (only ~/connect), so tapping
  // this button always calls connect regardless of which label shows. A
  // harmless no-op re-connect if already connected, but genuinely doesn't
  // disconnect — don't read DISCONNECT here as "will disconnect".
  bool connected = state.drivechain_status.status.rfind("connected", 0) == 0;
  std::array<std::string, 4> labels = {
    connected ? "DISCONNECT" : "CONNECT", "INFO", "FACE", "CALIBRATE"};
  for (size_t i = 0; i < r.size(); ++i) {
    fb.fillRect(r[i].x, r[i].y, r[i].w, r[i].h, colButton());
    fb.drawRect(r[i].x, r[i].y, r[i].w, r[i].h, colButtonBorder());
    centeredText(fb, r[i], labels[i], colWhite(), 3);
  }

  Rect status_area{r[0].x, r[3].y + r[3].h + 8, r[0].w, 22};
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

  // Web UI port (6240) matches run_stack.sh's python http.server. Grafana
  // port (3030) matches transfer.md's restore instructions for the
  // separate ~/grafana compose stack — that stack lives outside this repo
  // and isn't guaranteed to be running, so this is a best-effort address,
  // not a live-checked link (harmless to show even if unreachable).
  std::snprintf(buf, sizeof(buf), "WEB: %s:6240", state.ip_address.c_str());
  fb.drawText(x, y, buf, colWhite(), scale);
  y += line_h;

  std::snprintf(buf, sizeof(buf), "GRAFANA: %s:3030", state.ip_address.c_str());
  fb.drawText(x, y, buf, colWhite(), scale);
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

void renderCalibrate(Framebuffer & fb, const DisplayState & state)
{
  fb.clear(colBg());

  static const std::array<std::string, 4> kStepLabel = {"UP", "DOWN", "LEFT", "RIGHT"};
  int step = std::clamp(state.calib_step, 0, 3);
  const std::string & label = kStepLabel[step];

  Rect status_area{0, 140, fb.width(), 30};
  centeredText(
    fb, status_area, "CALIBRATE " + std::to_string(step + 1) + "/4 - TAP " + label, colWhite(), 2);

  // Only the current step's target is shown — showing all 4 at once
  // invites tapping the wrong one, and the point of a guided wizard is
  // there's exactly one unambiguous thing to do at each step.
  const int scale = 3;
  int tw = fb.textWidth(label, scale);
  int th = Framebuffer::textHeight(scale);
  int x = 0, y = 0;
  if (label == "UP") {
    x = (fb.width() - tw) / 2;
    y = 10;
  } else if (label == "DOWN") {
    x = (fb.width() - tw) / 2;
    y = fb.height() - th - 10;
  } else if (label == "LEFT") {
    x = 10;
    y = (fb.height() - th) / 2;
  } else {  // RIGHT
    x = fb.width() - tw - 10;
    y = (fb.height() - th) / 2;
  }
  fb.drawText(x, y, label, colFail(), scale);
}

}  // namespace mserve_display

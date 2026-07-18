#pragma once

#include <array>
#include <string>

#include <interfaces/msg/drive_motor_feedback.hpp>
#include <interfaces/msg/drive_status.hpp>

#include "mserve_display/framebuffer.hpp"

namespace mserve_display {

enum class Screen { Face, Menu, Info };

struct Rect
{
  int x = 0, y = 0, w = 0, h = 0;
  bool contains(int px, int py) const
  {
    return px >= x && px < x + w && py >= y && py < y + h;
  }
};

// Snapshot of everything the screens need to render, owned by DisplayNode
// and updated by its ROS callbacks. screens.cpp itself has no rclcpp
// dependency — these render functions are plain (Framebuffer&, const
// DisplayState&) -> void, independent of any active ROS node/executor.
struct DisplayState
{
  std::string ip_address = "...";
  std::string drivechain_state = "?";
  std::string base_state = "?";
  interfaces::msg::DriveStatus drivechain_status{};
  interfaces::msg::DriveStatus base_status{};
  interfaces::msg::DriveMotorFeedback motor_feedback{};

  double eye_direction = 0.0;  // smoothed, [-1, 1], negative = look right

  bool connect_in_flight = false;
  bool connect_result_valid = false;
  bool connect_result_success = false;
  std::string connect_result_message;
};

enum class MenuButton { None, Connect, Info, Face };

// Menu screen button layout — computed once from screen dimensions and
// reused by both rendering and hit-testing so they can never disagree.
std::array<Rect, 3> menuButtonRects(int screen_w, int screen_h);
MenuButton hitTestMenu(int x, int y, const std::array<Rect, 3> & button_rects);

void renderFace(Framebuffer & fb, const DisplayState & state);
void renderMenu(Framebuffer & fb, const DisplayState & state, const std::array<Rect, 3> & button_rects);
void renderInfo(Framebuffer & fb, const DisplayState & state);

}  // namespace mserve_display

#include <cstdio>
#include <cstdlib>

#include "mserve_joystick/joystick_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<mserve_joystick::JoystickNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "Fatal: %s\n", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

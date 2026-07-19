#include <rclcpp/rclcpp.hpp>

#include "mserve_sensehat/sensehat_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mserve_sensehat::SensehatNode>());
  rclcpp::shutdown();
  return 0;
}

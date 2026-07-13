#include <cstdlib>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "mserve_lidar/lidar_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<mserve_lidar::LidarNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
  } catch (const std::exception & e) {
    std::fprintf(stderr, "Fatal: %s\n", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

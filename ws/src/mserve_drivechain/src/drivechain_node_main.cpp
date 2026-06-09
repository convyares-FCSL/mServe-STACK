#include <cstdlib>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "mserve_drivechain/drivechain_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<mserve_drivechain::DrivechainNode>();

    // MultiThreaded: service callback (which runs tree ticks + sleeps) runs on
    // its own callback group thread, so the drive timer fires uninterrupted.
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

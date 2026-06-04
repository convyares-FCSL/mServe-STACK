#include <cstdlib>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "mserve_lifecycle_manager/lifecycle_manager.hpp"

// ==============================================================================
// Main entry point for the BaseNode.
// ==============================================================================

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<lifecyclemanager::LifecycleManager>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    node->build();
    executor.spin();
  } catch (const std::exception & e) {
    std::fprintf(stderr, "Fatal error: %s\n", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
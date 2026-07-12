#include <csignal>
#include <cstdlib>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "mserve_lifecycle_manager/lifecycle_manager.hpp"

// ==============================================================================
// Main entry point for the LifecycleManager.
// ==============================================================================

int main(int argc, char ** argv) {
  // Disable rclcpp's default SIGINT/SIGTERM handling — it calls
  // rclcpp::shutdown() directly from the signal handler, which invalidates
  // the ROS context before LifecycleManager gets a chance to run its
  // shutdown tree. Our handler just raises a flag; LifecycleManager::build()
  // runs the shutdown tree while the context is still valid, then shuts
  // down itself once that tree completes.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, [](int) { lifecyclemanager::LifecycleManager::requestShutdown(); });
  std::signal(SIGTERM, [](int) { lifecyclemanager::LifecycleManager::requestShutdown(); });

  try {
    auto node = std::make_shared<lifecyclemanager::LifecycleManager>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    node->build();
    executor.spin();
  } catch (const std::exception & e) {
    std::fprintf(stderr, "Fatal error: %s\n", e.what());
    if (rclcpp::ok()) rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  if (rclcpp::ok()) rclcpp::shutdown();
  return EXIT_SUCCESS;
}
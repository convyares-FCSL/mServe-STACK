#include <cstdlib>
#include <cstdio>

// TODO: update include and class name
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "hyfleet_subsystem/subsystem_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    // TODO: update namespace and class name
    auto node = std::make_shared<hyfleet_subsystem::SubsystemNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
  } catch (const std::exception & e) {
    std::fprintf(stderr, "Fatal error: %s\n", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

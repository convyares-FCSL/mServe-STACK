#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "compressor/compressor_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<compressor::CompressorNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  try {
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Compressor executor failed: %s", error.what());
  }

  executor.remove_node(node->get_node_base_interface());

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}

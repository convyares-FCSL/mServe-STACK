#include <algorithm>
#include <vector>
#include <hyfleet_compression/compression_node.hpp>

namespace hyfleet_compression {

// ==============================================================================
// Construction
// ==============================================================================

CompressionNode::CompressionNode(const rclcpp::NodeOptions & options): rclcpp_lifecycle::LifecycleNode("hyfleet_compression", options){
 
  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      return this->on_parameters(params);
    });

  RCLCPP_INFO(get_logger(), "Hyfleet_compression compression_node constructed");
}

// ==============================================================================
// Lifecycle Callbacks
// ==============================================================================

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressionNode::on_configure(const rclcpp_lifecycle::State &){
    RCLCPP_INFO(get_logger(), "Hyfleet_compression compression_node configured");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressionNode::on_activate(const rclcpp_lifecycle::State &){
    RCLCPP_INFO(get_logger(), "Hyfleet_compression compression_node activated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressionNode::on_deactivate(const rclcpp_lifecycle::State &){
    RCLCPP_INFO(get_logger(), "Hyfleet_compression compression_node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressionNode::on_cleanup(const rclcpp_lifecycle::State &){
    RCLCPP_INFO(get_logger(), "Hyfleet_compression compression_node unconfigured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressionNode::on_shutdown(const rclcpp_lifecycle::State &){
    RCLCPP_INFO(get_logger(), "Hyfleet_compression compression_node shutdown");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// ==============================================================================
// Core Logic
// ==============================================================================

rcl_interfaces::msg::SetParametersResult CompressionNode::on_parameters(const std::vector<rclcpp::Parameter> & params){
  (void) params;
  
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

}  // namespace hyfleet_compression

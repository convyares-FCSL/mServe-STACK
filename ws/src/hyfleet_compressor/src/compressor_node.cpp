#include <algorithm>
#include <vector>
#include <hyfleet_compressor/compressor_node.hpp>
#include "include/compressor_action.hpp"

namespace hyfleet_compressor {

// ==============================================================================
// Construction
// ==============================================================================

CompressorNode::CompressorNode(const rclcpp::NodeOptions & options): rclcpp_lifecycle::LifecycleNode("hyfleet_compressor", options){
 
  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      return this->on_parameters(params);
    });

  RCLCPP_INFO(get_logger(), "Hyfleet_Compressor Compressor_node constructed");
}

CompressorNode::~CompressorNode() = default;

// ==============================================================================
// Lifecycle Callbacks
// ==============================================================================

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressorNode::on_configure(const rclcpp_lifecycle::State &){

  RCLCPP_INFO(get_logger(), "Configuring compressor...");
  try {
    CommandLimitsConfig compressor_cmd_limits;
    action_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    compressor_action_ = std::make_unique<CompressorAction>(
      *this,
      "control_compressor",
      compressor_cmd_limits
    );

    compressor_action_->configure(action_callback_group_);
      
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to configure compressor: %s", error.what());
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Compressor configured");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressorNode::on_activate(const rclcpp_lifecycle::State &){
  compressor_action_->toggle_enable(true);
  
  RCLCPP_INFO(get_logger(), "Hyfleet_Compressor Compressor_node activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressorNode::on_deactivate(const rclcpp_lifecycle::State &){
  compressor_action_->toggle_enable(false);

  RCLCPP_INFO(get_logger(), "Hyfleet_Compressor Compressor_node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressorNode::on_cleanup(const rclcpp_lifecycle::State &){
  compressor_action_->unconfigure();
  
  RCLCPP_INFO(get_logger(), "Hyfleet_Compressor Compressor_node unconfigured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressorNode::on_shutdown(const rclcpp_lifecycle::State &){
    RCLCPP_INFO(get_logger(), "Hyfleet_Compressor Compressor_node shutdown");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// ==============================================================================
// Core Logic
// ==============================================================================

rcl_interfaces::msg::SetParametersResult CompressorNode::on_parameters(const std::vector<rclcpp::Parameter> & params){
  (void) params;
  
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

}  // namespace hyfleet_compressor

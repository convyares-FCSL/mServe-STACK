#include <algorithm>
#include <vector>
#include <hyfleet_booster/booster_node.hpp>
#include "include/booster_action.hpp"

namespace hyfleet_booster {

// ==============================================================================
// Construction
// ==============================================================================

BoosterNode::BoosterNode(const rclcpp::NodeOptions & options): rclcpp_lifecycle::LifecycleNode("booster_node", options){
 
  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      return this->on_parameters(params);
    });

  RCLCPP_INFO(get_logger(), "Hyfleet_booster booster_node constructed");
}

BoosterNode::~BoosterNode() = default;

// ==============================================================================
// Lifecycle Callbacks
// ==============================================================================

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BoosterNode::on_configure(const rclcpp_lifecycle::State &){
  try {
    action_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    booster_action_ = std::make_unique<BoosterAction>(
      *this,
      "control_booster"
    );

    booster_action_->configure(action_callback_group_);
      
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to configure compressor: %s", error.what());
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "booster configured");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BoosterNode::on_activate(const rclcpp_lifecycle::State &){
  booster_action_->toggle_enable(true);
  
  RCLCPP_INFO(get_logger(), "Hyfleet_booster booster_node activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BoosterNode::on_deactivate(const rclcpp_lifecycle::State &){
  booster_action_->toggle_enable(false);
  
  RCLCPP_INFO(get_logger(), "Hyfleet_booster booster_node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BoosterNode::on_cleanup(const rclcpp_lifecycle::State &){
   booster_action_->unconfigure();
  
  RCLCPP_INFO(get_logger(), "Hyfleet_booster booster_node unconfigured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BoosterNode::on_shutdown(const rclcpp_lifecycle::State &){
    RCLCPP_INFO(get_logger(), "Hyfleet_booster booster_node shutdown");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// ==============================================================================
// Core Logic
// ==============================================================================

rcl_interfaces::msg::SetParametersResult BoosterNode::on_parameters(const std::vector<rclcpp::Parameter> & params){
  (void) params;
  
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

}  // namespace hyfleet_booster

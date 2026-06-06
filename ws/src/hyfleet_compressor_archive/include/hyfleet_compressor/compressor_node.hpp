#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// ==============================================================================
// CompressorNode: a lifecycle node that compresses data.
// ==============================================================================

namespace hyfleet_compressor {

class CompressorAction;

class CompressorNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit CompressorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CompressorNode() override;
  
protected:
  // Lifecycle states
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  // Param change 
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> & params);
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Action Server
  std::unique_ptr<CompressorAction> compressor_action_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
 
};

} // namespace hyfleet_compressor

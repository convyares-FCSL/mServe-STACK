#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include "mserve_interfaces/action/control_compressor.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace hyfleet_compressor {
using ControlCompressor = mserve_interfaces::action::ControlCompressor;
using GoalHandleControlCompressor = rclcpp_action::ServerGoalHandle<ControlCompressor>;

class CompressorAction;

class CompressorNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit CompressorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CompressorNode() override;

protected:
  //Lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  // Behavoir tree
  struct OpSlot {
    std::shared_ptr<GoalHandleControlCompressor> goal;
    std::shared_ptr<BT::Blackboard>              blackboard;
    std::optional<BT::Tree>                      tree;
    uint8_t                                      mode = 0;
    uint8_t                                      target = 0;
    bool active() const { return goal != nullptr; }
  };
private:
  // Param handling (found in compressor_params.cpp)
  void declare_params();
  void load_params();
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> & params);
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Action server
  std::unique_ptr<CompressorAction> compressor_action_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;

  // Behavior tree
  BT::BehaviorTreeFactory factory_;
  std::shared_ptr<BT::Blackboard> shared_blackboard_;
  std::array<OpSlot, 2> ops_{};
  void register_bt_nodes();
  void build_bt_trees();
  bool start_op(OpSlot & slot, uint8_t command, uint8_t target, uint8_t goal_mode, double target_pressure);
  void tick_tree_once();

  // Goal and tick
  void on_compressor_goal_accepted(std::shared_ptr<GoalHandleControlCompressor> goal_handle);
  void set_tick_timer(bool enable);
  rclcpp::TimerBase::SharedPtr tick_timer_;

  // ROS client node for BT RosActionNodes — requires rclcpp::Node, not LifecycleNode
  std::shared_ptr<rclcpp::Node> bt_node_;

  // Goal validation limits (loaded from params in on_configure)
  double min_pressure_bar_;
  double max_pressure_bar_;
  double performance_cpm_;
  double eco_cpm_;
  double default_speed_rpm_;
};

} // namespace hyfleet_compressor

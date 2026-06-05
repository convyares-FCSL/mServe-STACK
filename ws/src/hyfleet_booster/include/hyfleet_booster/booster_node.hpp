#pragma once

#include <memory>
#include <string>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include "mserve_interfaces/action/control_booster.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace hyfleet_booster {
using ControlBooster = mserve_interfaces::action::ControlBooster;
using GoalHandleControlBooster = rclcpp_action::ServerGoalHandle<ControlBooster>;

class BoosterAction;

class BoosterNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit BoosterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BoosterNode() override;

protected:
  //Lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  // Param handling
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> & params);
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Action server
  std::unique_ptr<BoosterAction> booster_action_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
  std::shared_ptr<GoalHandleControlBooster> active_goal_;
  
  // The behavior tree instance
  BT::Tree tree_;
  BT::BehaviorTreeFactory factory_;

  // BT execution
  void on_booster_goal_accepted(std::shared_ptr<GoalHandleControlBooster> goal_handle);
  void start_tick_timer();
  void stop_tick_timer();
  void tick_tree_once();
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

} // namespace hyfleet_booster

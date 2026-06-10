#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// TODO: replace with your action header, e.g.:
//   #include "interfaces/action/control_<name>.hpp"
#include "interfaces/action/control_subsystem.hpp"

// TODO: rename namespace to hyfleet_<name>
namespace hyfleet_subsystem {

// TODO: replace ControlSubsystem with your action type
using ControlSubsystem = interfaces::action::ControlSubsystem;
using GoalHandleControlSubsystem = rclcpp_action::ServerGoalHandle<ControlSubsystem>;

class SubsystemAction;

// TODO: rename class to <Name>Node
class SubsystemNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit SubsystemNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SubsystemNode() override;

protected:
  //Lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  // Param handling (subsystem_params.cpp)
  void declare_params();
  void load_params();
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> & params);
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Action server
  std::unique_ptr<SubsystemAction> subsystem_action_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
  std::shared_ptr<GoalHandleControlSubsystem> active_goal_;

  // Behavior tree
  BT::BehaviorTreeFactory factory_;
  std::shared_ptr<BT::Blackboard> blackboard_;
  // TODO: set array size to number of commands (one tree per command)
  std::array<BT::Tree, 1> trees_;
  BT::Tree* active_tree_ = nullptr;
  void register_bt_nodes();
  void build_bt_trees();
  bool select_tree(uint8_t command);
  void tick_tree_once();

  // Goal and tick
  void on_subsystem_goal_accepted(std::shared_ptr<GoalHandleControlSubsystem> goal_handle);
  void set_tick_timer(bool enable);
  rclcpp::TimerBase::SharedPtr tick_timer_;

  // ROS client node for BT RosActionNodes / RosServiceNodes
  std::shared_ptr<rclcpp::Node> bt_node_;

  // TODO: add per-instance validation limits loaded from params, e.g.:
  //   double min_pressure_bar_;
  //   double max_pressure_bar_;
};

} // namespace hyfleet_subsystem

#include <chrono>

#include <hyfleet_subsystem/subsystem_node.hpp>
#include <hyfleet_subsystem/subsystem_limits.hpp>
#include "include/subsystem_action.hpp"
#include "include/subsystem_bt_nodes.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// TODO: rename namespace to hyfleet_<name>
namespace hyfleet_subsystem {
using namespace std::chrono_literals;

// ==============================================================================
// Construction
// ==============================================================================

SubsystemNode::SubsystemNode(const rclcpp::NodeOptions & options)
// TODO: set ROS node name, e.g. "hyfleet_<name>" or "<name>_node"
: rclcpp_lifecycle::LifecycleNode("subsystem_node", options)
{
  declare_params();

  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      return this->on_parameters(params);
    });

  RCLCPP_INFO(get_logger(), "subsystem_node constructed");
}

SubsystemNode::~SubsystemNode() = default;

// ==============================================================================
// Lifecycle Callbacks
// — these are boilerplate; only change log messages and action/tree names
// ==============================================================================

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SubsystemNode::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    action_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // TODO: set action server topic name, e.g. "~/control_<name>"
    subsystem_action_ = std::make_unique<SubsystemAction>(*this, "~/control_subsystem");
    subsystem_action_->configure(action_callback_group_);
    subsystem_action_->set_goal_callback(
      [this](auto goal_handle) { this->on_subsystem_goal_accepted(goal_handle); });

    bt_node_    = std::make_shared<rclcpp::Node>(std::string(get_name()) + "_bt");
    blackboard_ = BT::Blackboard::create();

    // TODO: if this node owns a telemetry subscription, create the cache and
    // subscription here, then place the cache on the blackboard. See
    // hyfleet_booster/src/booster_node.cpp for the BoosterTelemetryCache pattern.

    load_params();
    register_bt_nodes();
    build_bt_trees();

  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to configure subsystem: %s", error.what());
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "subsystem_node configured");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SubsystemNode::on_activate(const rclcpp_lifecycle::State &)
{
  subsystem_action_->toggle_enable(true);
  RCLCPP_INFO(get_logger(), "subsystem_node activated");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SubsystemNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  subsystem_action_->toggle_enable(false);
  set_tick_timer(false);
  if (active_tree_) { active_tree_->haltTree(); active_tree_ = nullptr; }
  if (active_goal_) {
    subsystem_action_->abort_goal(active_goal_, "node deactivated");
    active_goal_.reset();
  }
  RCLCPP_INFO(get_logger(), "subsystem_node deactivated");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SubsystemNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (active_goal_) {
    subsystem_action_->abort_goal(active_goal_, "node cleanup");
    active_goal_.reset();
  }
  set_tick_timer(false);
  if (active_tree_) { active_tree_->haltTree(); active_tree_ = nullptr; }
  trees_ = {};

  // TODO: reset telemetry subscription and cache here if used (before bt_node_ reset)

  bt_node_.reset();
  if (subsystem_action_) { subsystem_action_->unconfigure(); }

  RCLCPP_INFO(get_logger(), "subsystem_node cleaned up");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SubsystemNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  set_tick_timer(false);
  if (active_tree_) { active_tree_->haltTree(); active_tree_ = nullptr; }
  if (active_goal_ && subsystem_action_) {
    subsystem_action_->abort_goal(active_goal_, "node shutdown");
    active_goal_.reset();
  }
  RCLCPP_INFO(get_logger(), "subsystem_node shutdown");
  return CallbackReturn::SUCCESS;
}

// ==============================================================================
// Behavior Tree
// ==============================================================================

void SubsystemNode::register_bt_nodes()
{
  // TODO: register BT nodes here.
  //
  // RosServiceNode / RosActionNode (need bt_node_ for ROS comms):
  //   factory_.registerBuilder<MyServiceNode>("MyServiceNode",
  //     [this](const std::string& name, const BT::NodeConfig& config) {
  //       return std::make_unique<MyServiceNode>(name, config, BT::RosNodeParams(bt_node_));
  //     });
  //
  // StatefulActionNode / ConditionNode (no ROS comms needed):
  //   factory_.registerNodeType<MyConditionNode>("MyConditionNode");
}

void SubsystemNode::build_bt_trees()
{
  // TODO: update package name and add/remove trees to match your command set.
  // One tree per command, array index = command - 1.
  const std::string base =
    ament_index_cpp::get_package_share_directory("hyfleet_subsystem") + "/trees/";
  trees_[0] = factory_.createTreeFromFile(base + "main_tree.xml", blackboard_);
}

bool SubsystemNode::select_tree(uint8_t command)
{
  // TODO: replace with your action's command constants, e.g.:
  //   case ControlSubsystem::Goal::START: active_tree_ = &trees_[0]; return true;
  switch (command) {
    case 1: active_tree_ = &trees_[0]; return true;
    default: return false;
  }
}

void SubsystemNode::tick_tree_once()
{
  if (!active_goal_) { set_tick_timer(false); return; }
  if (!active_tree_) { set_tick_timer(false); return; }

  const auto status = active_tree_->tickOnce();

  // Publish feedback each tick. BT nodes write feedback keys to the blackboard;
  // read them here and forward to the action client.
  // TODO: replace blackboard keys and Feedback fields to match your action.
  {
    auto fb = std::make_shared<ControlSubsystem::Feedback>();
    double pressure = 0.0;
    double pct      = 0.0;
    (void)blackboard_->get("pressure",          pressure);
    (void)blackboard_->get("percent_complete",   pct);
    fb->pressure         = pressure;
    fb->percent_complete = pct;
    active_goal_->publish_feedback(fb);
  }

  if (status == BT::NodeStatus::RUNNING) { return; }

  if (status == BT::NodeStatus::SUCCESS) {
    subsystem_action_->succeed_goal(active_goal_);
    RCLCPP_INFO(get_logger(), "Goal completed successfully");
  } else if (status == BT::NodeStatus::FAILURE) {
    subsystem_action_->abort_goal(active_goal_, "tree failed");
    RCLCPP_WARN(get_logger(), "Goal aborted — tree returned FAILURE");
  } else {
    subsystem_action_->abort_goal(active_goal_, "tree returned unexpected status");
    RCLCPP_ERROR(get_logger(), "Tree returned unexpected status");
  }

  active_tree_->haltTree();
  active_tree_ = nullptr;
  active_goal_.reset();
  set_tick_timer(false);
}

// ==============================================================================
// Goal and tick
// ==============================================================================

void SubsystemNode::on_subsystem_goal_accepted(
  std::shared_ptr<GoalHandleControlSubsystem> goal_handle)
{
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "Received null goal handle");
    return;
  }

  if (active_goal_) {
    subsystem_action_->abort_goal(active_goal_, "replaced by newer goal");
    active_goal_.reset();
  }
  if (active_tree_) { active_tree_->haltTree(); }

  const auto goal = goal_handle->get_goal();

  if (!select_tree(goal->command)) {
    RCLCPP_ERROR(get_logger(), "Unknown command: %d", goal->command);
    subsystem_action_->abort_goal(goal_handle, "unknown command");
    return;
  }

  // ==============================================================================
  // TODO: validate goal fields against limits, e.g.:
  //   if (goal->target_pressure < min_pressure_bar_ || goal->target_pressure > max_pressure_bar_) {
  //     subsystem_action_->abort_goal(goal_handle, "target_pressure out of bounds");
  //     return;
  //   }
  // ==============================================================================

  // ==============================================================================
  // TODO: write validated goal fields to blackboard for BT tree consumption, e.g.:
  //   blackboard_->set("target_pressure", goal->target_pressure);
  // ==============================================================================

  active_goal_ = std::move(goal_handle);
  set_tick_timer(true);
  RCLCPP_INFO(get_logger(), "Goal accepted, BT tick timer started");
}

void SubsystemNode::set_tick_timer(bool enable)
{
  if (tick_timer_) { tick_timer_->cancel(); tick_timer_.reset(); }
  if (enable) {
    tick_timer_ = create_wall_timer(100ms, [this]() { this->tick_tree_once(); });
  }
}

}  // namespace hyfleet_subsystem

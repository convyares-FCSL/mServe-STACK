#include <algorithm>
#include <vector>
#include <hyfleet_booster/booster_node.hpp>
#include "include/booster_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>

namespace hyfleet_booster {
using namespace std::chrono_literals;

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
    // Create action booster and configure
    action_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    booster_action_ = std::make_unique<BoosterAction>(*this, "~/control_booster");
    booster_action_->configure(action_callback_group_);
      
    booster_action_->set_goal_callback([this](auto goal_handle) { this->on_booster_goal_accepted(goal_handle); });

    // Load tree 
    std::string tree_path = ament_index_cpp::get_package_share_directory("hyfleet_booster")  + "/trees/booster.xml";
    tree_ = factory_.createTreeFromFile(tree_path);

  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to configure booster: %s", error.what());
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

  stop_tick_timer();
  tree_.haltTree();

  if (active_goal_) {
    booster_action_->abort_goal(active_goal_, "booster node deactivated");
    active_goal_.reset();
  }

  RCLCPP_INFO(get_logger(), "Hyfleet_booster booster_node deactivated");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BoosterNode::on_cleanup(const rclcpp_lifecycle::State &){
  if (active_goal_) {
      booster_action_->abort_goal(active_goal_, "node cleanup");
      active_goal_.reset();
  }

  stop_tick_timer();
  tree_.haltTree();
  tree_ = BT::Tree{};

  if (booster_action_) {
    booster_action_->unconfigure();
  }
  
  RCLCPP_INFO(get_logger(), "Hyfleet_booster booster_node unconfigured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BoosterNode::on_shutdown(const rclcpp_lifecycle::State &){
  stop_tick_timer();
  tree_.haltTree();

  if (active_goal_ && booster_action_) {
    booster_action_->abort_goal(active_goal_, "booster node shutdown");
    active_goal_.reset();
  }

  RCLCPP_INFO(get_logger(), "Hyfleet_booster booster_node shutdown");
  return CallbackReturn::SUCCESS;
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

void BoosterNode::on_booster_goal_accepted(std::shared_ptr<GoalHandleControlBooster> goal_handle) {
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "Received null booster goal handle");
    return;
  }

  if (active_goal_) {
    booster_action_->abort_goal(active_goal_, "booster goal replaced by newer goal");
    active_goal_.reset();
  }

  tree_.haltTree();

  active_goal_ = std::move(goal_handle);
  start_tick_timer();

  RCLCPP_INFO(get_logger(), "Booster goal accepted, BT tick timer started");
}

void BoosterNode::start_tick_timer() {
  stop_tick_timer();

  tick_timer_ = create_wall_timer(100ms, [this]() { this->tick_tree_once(); });
}

void BoosterNode::stop_tick_timer() {
  if (tick_timer_) {  tick_timer_->cancel(); }
}

void BoosterNode::tick_tree_once(){
  if (!active_goal_) {
    stop_tick_timer();
    return;
  }

  const auto status = tree_.tickOnce();

  if (status == BT::NodeStatus::RUNNING) { return; }

  if (status == BT::NodeStatus::SUCCESS) {
    booster_action_->succeed_goal(active_goal_);
    RCLCPP_INFO(get_logger(), "Booster goal completed successfully");
  } else if (status == BT::NodeStatus::FAILURE) {
    booster_action_->abort_goal(active_goal_, "booster tree failed");
    RCLCPP_WARN(get_logger(), "Booster goal aborted because tree returned FAILURE");
  } else {
    booster_action_->abort_goal(active_goal_, "booster tree returned unexpected status");
    RCLCPP_ERROR(get_logger(), "Booster tree returned unexpected status");
  }

  tree_.haltTree();
  active_goal_.reset();
  stop_tick_timer();
}

}  // namespace hyfleet_booster
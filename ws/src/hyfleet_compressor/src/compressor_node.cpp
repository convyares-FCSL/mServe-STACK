#include <chrono>

#include <hyfleet_compressor/compressor_node.hpp>
#include <hyfleet_compressor/compressor_limits.hpp>
#include "include/compressor_action.hpp"
#include "include/compressor_bt_nodes.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace hyfleet_compressor {
using namespace std::chrono_literals;

// ==============================================================================
// Construction
// ==============================================================================

CompressorNode::CompressorNode(const rclcpp::NodeOptions & options) : rclcpp_lifecycle::LifecycleNode("hyfleet_compression", options) {
  declare_params();

  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      return this->on_parameters(params);
    });

  RCLCPP_INFO(get_logger(), "hyfleet_compression constructed");
}

CompressorNode::~CompressorNode() = default;

// ==============================================================================
// Lifecycle Callbacks
// ==============================================================================

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressorNode::on_configure(const rclcpp_lifecycle::State &) {
  try {
    // Action server :Create, configure, goal callback
    action_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    compressor_action_ = std::make_unique<CompressorAction>(*this, "~/control_compressor");
    compressor_action_->configure(action_callback_group_);
    compressor_action_->set_goal_callback( [this](auto goal_handle) { this->on_compressor_goal_accepted(goal_handle); });

    // BT node : Create blackboard, load params, register nodes and build trees
    bt_node_ = std::make_shared<rclcpp::Node>(std::string(get_name()) + "_bt");
    blackboard_ = BT::Blackboard::create();

    load_params();
    register_bt_nodes();
    build_bt_trees();

  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to configure compressor: %s", error.what());
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "hyfleet_compression configured");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressorNode::on_activate(const rclcpp_lifecycle::State &) {
  compressor_action_->toggle_enable(true);

  RCLCPP_INFO(get_logger(), "hyfleet_compression activated");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressorNode::on_deactivate(const rclcpp_lifecycle::State &) {
  compressor_action_->toggle_enable(false);

  set_tick_timer(false);
  if (active_tree_) { active_tree_->haltTree(); active_tree_ = nullptr; }

  if (active_goal_) {
    compressor_action_->abort_goal(active_goal_, "coordinator node deactivated");
    active_goal_.reset();
  }

  RCLCPP_INFO(get_logger(), "hyfleet_compression deactivated");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressorNode::on_cleanup(const rclcpp_lifecycle::State &) {
  if (active_goal_) {
    compressor_action_->abort_goal(active_goal_, "node cleanup");
    active_goal_.reset();
  }

  set_tick_timer(false);
  if (active_tree_) { active_tree_->haltTree(); active_tree_ = nullptr; }
  trees_ = {};

  bt_node_.reset();

  if (compressor_action_) {
    compressor_action_->unconfigure();
  }

  RCLCPP_INFO(get_logger(), "hyfleet_compression cleaned up");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressorNode::on_shutdown(const rclcpp_lifecycle::State &) {
  set_tick_timer(false);
  if (active_tree_) { active_tree_->haltTree(); active_tree_ = nullptr; }

  if (active_goal_ && compressor_action_) {
    compressor_action_->abort_goal(active_goal_, "coordinator node shutdown");
    active_goal_.reset();
  }

  RCLCPP_INFO(get_logger(), "hyfleet_compression shutdown");
  return CallbackReturn::SUCCESS;
}

// ==============================================================================
// Behavior Tree
// ==============================================================================

void CompressorNode::register_bt_nodes() {
  // Stage 3: register RosActionNode wrappers here (BoostLow, BoostHigh).
  // See src/include/compressor_bt_nodes.hpp.
}

void CompressorNode::build_bt_trees() {
  const std::string base = ament_index_cpp::get_package_share_directory("hyfleet_compressor") + "/trees/";
  trees_[0] = factory_.createTreeFromFile(base + "start_tree.xml",     blackboard_);
  trees_[1] = factory_.createTreeFromFile(base + "stop_tree.xml",      blackboard_);
  trees_[2] = factory_.createTreeFromFile(base + "safe_stop_tree.xml", blackboard_);
}

bool CompressorNode::select_tree(uint8_t command) {
  switch (command) {
    case ControlCompressor::Goal::START:     active_tree_ = &trees_[0]; return true;
    case ControlCompressor::Goal::STOP:      active_tree_ = &trees_[1]; return true;
    case ControlCompressor::Goal::SAFE_STOP: active_tree_ = &trees_[2]; return true;
    default: return false;
  }
}

void CompressorNode::tick_tree_once() {
  if (!active_goal_) { set_tick_timer(false); return; }
  if (!active_tree_) { set_tick_timer(false); return; }

  const auto status = active_tree_->tickOnce();

  // Publish feedback each tick. BoostLow / BoostHigh write pressure and percent_complete
  // to the blackboard via onFeedback() once wired in Stage 3. Defaults to 0.0 until then.
  {
    auto fb = std::make_shared<ControlCompressor::Feedback>();
    double pressure = 0.0;
    double pct      = 0.0;
    (void)blackboard_->get("pressure",         pressure);
    (void)blackboard_->get("percent_complete",  pct);
    fb->pressure         = pressure;
    fb->percent_complete = pct;
    active_goal_->publish_feedback(fb);
  }

  if (status == BT::NodeStatus::RUNNING) { return; }

  if (status == BT::NodeStatus::SUCCESS) {
    compressor_action_->succeed_goal(active_goal_);
    RCLCPP_INFO(get_logger(), "Coordinator goal completed successfully");
  } else if (status == BT::NodeStatus::FAILURE) {
    compressor_action_->abort_goal(active_goal_, "coordinator tree failed");
    RCLCPP_WARN(get_logger(), "Coordinator goal aborted — tree returned FAILURE");
  } else {
    compressor_action_->abort_goal(active_goal_, "coordinator tree returned unexpected status");
    RCLCPP_ERROR(get_logger(), "Coordinator tree returned unexpected status");
  }

  active_tree_->haltTree();
  active_tree_ = nullptr;
  active_goal_.reset();
  set_tick_timer(false);
}

// ==============================================================================
// Goal and tick
// ==============================================================================

void CompressorNode::on_compressor_goal_accepted( std::shared_ptr<GoalHandleControlCompressor> goal_handle) {
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "Received null coordinator goal handle");
    return;
  }

  // Abort existing goal
  if (active_goal_) {
    compressor_action_->abort_goal(active_goal_, "coordinator goal replaced by newer goal");
    active_goal_.reset();
  }

  // Abort existing tree
  if (active_tree_) {
    active_tree_->haltTree();
  }

  const auto goal = goal_handle->get_goal();

  // Select tree
  if (!select_tree(goal->command)) {
    RCLCPP_ERROR(get_logger(), "Unknown command: %d", goal->command);
    compressor_action_->abort_goal(goal_handle, "unknown command");
    return;
  }

  // Validate target
  if (goal->target != ControlCompressor::Goal::LOW_BOOSTER &&
      goal->target != ControlCompressor::Goal::HIGH_BOOSTER &&
      goal->target != ControlCompressor::Goal::SYNC_BOOSTERS) {
    compressor_action_->abort_goal(goal_handle, "unknown target");
    return;
  }

  // Validate pressure
  if (goal->target_pressure < min_pressure_bar_ || goal->target_pressure > max_pressure_bar_) {
    compressor_action_->abort_goal(goal_handle, "target_pressure out of bounds");
    return;
  }

  // Write goal fields to blackboard for BT tree consumption
  blackboard_->set("command",          static_cast<int>(goal->command));
  blackboard_->set("target",           static_cast<int>(goal->target));
  blackboard_->set("mode",             static_cast<int>(goal->mode));
  blackboard_->set("target_pressure",  goal->target_pressure);

  active_goal_ = std::move(goal_handle);
  set_tick_timer(true);

  RCLCPP_INFO(get_logger(), "Coordinator goal accepted, BT tick timer started");
}

void CompressorNode::set_tick_timer(bool enable)
{
  if (tick_timer_) { tick_timer_->cancel(); tick_timer_.reset(); }
  if (enable) {
    tick_timer_ = create_wall_timer(100ms, [this]() { this->tick_tree_once(); });
  }
}

}  // namespace hyfleet_compressor

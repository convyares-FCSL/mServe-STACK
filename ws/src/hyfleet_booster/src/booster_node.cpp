#include <algorithm>
#include <chrono>

#include <hyfleet_booster/booster_node.hpp>
#include <hyfleet_booster/booster_limits.hpp>
#include "include/booster_action.hpp"
#include "include/booster_bt_actions.hpp"
#include "include/booster_bt_conditions.hpp"
#include "include/booster_telemetry_cache.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace hyfleet_booster {
using namespace std::chrono_literals;

// ==============================================================================
// Construction
// ==============================================================================

BoosterNode::BoosterNode(const rclcpp::NodeOptions & options): rclcpp_lifecycle::LifecycleNode("booster_node", options){
  declare_params();

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
    // Action server :Create, configure, goal callback
    action_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    booster_action_ = std::make_unique<BoosterAction>(*this, "~/control_booster");
    booster_action_->configure(action_callback_group_);
    booster_action_->set_goal_callback([this](auto goal_handle) { this->on_booster_goal_accepted(goal_handle); });

    // BT node : Create blackboard, load params, register nodes and build trees
    bt_node_ = std::make_shared<rclcpp::Node>(std::string(get_name()) + "_bt");
    blackboard_ = BT::Blackboard::create();

    // Telemetry: node owns the subscription; cache is shared with BT nodes via blackboard
    telemetry_cache_ = std::make_shared<BoosterTelemetryCache>();
    blackboard_->set("telemetry_cache", telemetry_cache_);
    telemetry_sub_ = create_subscription<mserve_interfaces::msg::CompressorTelemetry>(
      "compressor_telemetry", 10,
      [this](std::shared_ptr<const mserve_interfaces::msg::CompressorTelemetry> msg) {
        telemetry_cache_->update(msg, now());
      });

    load_params();
    register_bt_nodes();
    build_bt_trees();

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

  set_tick_timer(false);
  if (active_tree_) { active_tree_->haltTree(); active_tree_ = nullptr; }

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

  set_tick_timer(false);
  if (active_tree_) { active_tree_->haltTree(); active_tree_ = nullptr; }
  trees_ = {};

  telemetry_sub_.reset();
  telemetry_cache_.reset();
  bt_node_.reset();

  if (booster_action_) {
    booster_action_->unconfigure();
  }
  
  RCLCPP_INFO(get_logger(), "Hyfleet_booster booster_node unconfigured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BoosterNode::on_shutdown(const rclcpp_lifecycle::State &){
  set_tick_timer(false);
  if (active_tree_) { active_tree_->haltTree(); active_tree_ = nullptr; }

  if (active_goal_ && booster_action_) {
    booster_action_->abort_goal(active_goal_, "booster node shutdown");
    active_goal_.reset();
  }

  RCLCPP_INFO(get_logger(), "Hyfleet_booster booster_node shutdown");
  return CallbackReturn::SUCCESS;
}

// ==============================================================================
// Behavoir Tree
// ==============================================================================

void BoosterNode::register_bt_nodes(){
    // Register Actions
    factory_.registerBuilder<StartVFD>("StartVFD",
      [this](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<StartVFD>(name, config, BT::RosNodeParams(bt_node_));
    });
    factory_.registerBuilder<StopVFD>("StopVFD",
      [this](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<StopVFD>(name, config, BT::RosNodeParams(bt_node_));
    });
    factory_.registerBuilder<SetPCSV>("SetPCSV",
      [this](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<SetPCSV>(name, config, BT::RosNodeParams(bt_node_));
    });
    factory_.registerBuilder<ControlSV>("ControlSV",
      [this](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<ControlSV>(name, config, BT::RosNodeParams(bt_node_));
    });

  
    // Register Conditions — plain ConditionNode, read telemetry from shared cache on blackboard
    factory_.registerNodeType<InletPressureStable>("InletPressureStable");
    factory_.registerNodeType<VFDAtSpeed>("VFDAtSpeed");
    factory_.registerNodeType<VFDStopped>("VFDStopped");
    factory_.registerNodeType<OutletAtPressure>("OutletAtPressure");
    factory_.registerNodeType<InletPressureSafe>("InletPressureSafe");
}

void BoosterNode::build_bt_trees(){
    const std::string base = ament_index_cpp::get_package_share_directory("hyfleet_booster") + "/trees/";
    trees_[0] = factory_.createTreeFromFile(base + "start_tree.xml", blackboard_);
    trees_[1] = factory_.createTreeFromFile(base + "start_idle_tree.xml", blackboard_);
    trees_[2] = factory_.createTreeFromFile(base + "stop_tree.xml", blackboard_);
    trees_[3] = factory_.createTreeFromFile(base + "stop_force_tree.xml", blackboard_);
}

bool BoosterNode::select_tree(uint8_t command) {
  switch (command) {
      case ControlBooster::Goal::START:      active_tree_ = &trees_[0]; return true;
      case ControlBooster::Goal::START_IDLE: active_tree_ = &trees_[1]; return true;
      case ControlBooster::Goal::STOP:       active_tree_ = &trees_[2]; return true;
      case ControlBooster::Goal::SAFE_STOP:  active_tree_ = &trees_[3]; return true;
      default: return false;
  }
}

void BoosterNode::tick_tree_once(){
  if (!active_goal_) { set_tick_timer(false); return; }
  if (!active_tree_) { set_tick_timer(false); return; }

  const auto status = active_tree_->tickOnce();

  // Publish feedback every tick — pressure from cache, percent_complete for start goals only
  {
    auto [msg, stamp] = telemetry_cache_->latest();
    if (msg) {
      int outlet_idx = 0;
      double target  = 1.0;
      if (!blackboard_->get("outlet_pt_index", outlet_idx) ||
          !blackboard_->get("target_pressure", target)) { return; }
      const double pressure = msg->pt_bar[outlet_idx];
      const uint8_t cmd = active_goal_->get_goal()->command;
      const bool is_start = (cmd == ControlBooster::Goal::START ||
                             cmd == ControlBooster::Goal::START_IDLE);
      auto fb = std::make_shared<ControlBooster::Feedback>();
      fb->pressure         = pressure;
      fb->percent_complete = is_start ? std::clamp(pressure / target * 100.0, 0.0, 100.0) : 0.0;
      active_goal_->publish_feedback(fb);
    }
  }

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

  active_tree_->haltTree();
  active_tree_ = nullptr;
  active_goal_.reset();
  set_tick_timer(false);
}

// ==============================================================================
// Goal and tick
// ==============================================================================

void BoosterNode::on_booster_goal_accepted( std::shared_ptr<GoalHandleControlBooster> goal_handle) {
  if (!goal_handle) { RCLCPP_WARN(get_logger(), 
    "Received null booster goal handle"); 
    return; 
  }

  // Abort existing goal
  if (active_goal_) {
    booster_action_->abort_goal(active_goal_, "booster goal replaced by newer goal");
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
    booster_action_->abort_goal(goal_handle, "unknown command");
    return;
  }

  // Validate goal fields
  if (goal->target_pressure < min_pressure_bar_ || goal->target_pressure > max_pressure_bar_) {
    booster_action_->abort_goal(goal_handle, "target_pressure out of bounds");
    return;
  }

  if (goal->cpm < cpm_min || goal->cpm > cpm_max) {
    booster_action_->abort_goal(goal_handle, "cpm out of bounds");
    return;
  }

  if (goal->speed_rpm < speed_min || goal->speed_rpm > speed_max) {
    booster_action_->abort_goal(goal_handle, "speed_rpm out of bounds");
    return;
  }

  // Write validated goal fields to blackboard
  blackboard_->set("target_pressure", goal->target_pressure);
  blackboard_->set("cpm", goal->cpm);
  blackboard_->set("speed_rpm", goal->speed_rpm);

  // Start Timer
  active_goal_ = std::move(goal_handle);
  set_tick_timer(true);

  RCLCPP_INFO(get_logger(), "Booster goal accepted, BT tick timer started");
}

void BoosterNode::set_tick_timer(bool enable) {
  if (tick_timer_) { tick_timer_->cancel(); tick_timer_.reset(); }
  if (enable) {
    tick_timer_ = create_wall_timer(100ms, [this]() { this->tick_tree_once(); });
  }
}

}  // namespace hyfleet_booster
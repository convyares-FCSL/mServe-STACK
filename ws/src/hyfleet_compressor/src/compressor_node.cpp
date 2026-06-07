#include <chrono>

#include <hyfleet_compressor/compressor_node.hpp>
#include <hyfleet_compressor/compressor_limits.hpp>
#include "include/compressor_action.hpp"
#include "include/compressor_bt_nodes.hpp"
#include "include/compressor_telemetry_cache.hpp"
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
  ops_ = {};
  factory_.reset();
  factory_ = std::make_unique<BT::BehaviorTreeFactory>();
  if (compressor_action_) { compressor_action_->unconfigure(); compressor_action_.reset(); }
  if (action_callback_group_) { action_callback_group_.reset(); }
  bt_node_.reset();
  shared_blackboard_.reset();

  try {
    // Setup action server
    action_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    compressor_action_ = std::make_unique<CompressorAction>(*this, "~/control_compressor");
    compressor_action_->configure(action_callback_group_);
    compressor_action_->set_goal_callback( [this](auto goal_handle) { this->on_compressor_goal_accepted(goal_handle); });

    set_mode_srv_ = create_service<SetMode>(
      "~/set_mode",
      [this](const std::shared_ptr<SetMode::Request> req, std::shared_ptr<SetMode::Response> resp) {
        if (req->mode != SetMode::Request::PERFORMANCE && req->mode != SetMode::Request::ECO) {
          resp->success = false;
          resp->message = "unknown mode — use PERFORMANCE(1) or ECO(2)";
          return;
        }
        current_mode_ = req->mode;
        resp->success = true;
        resp->message = (current_mode_ == SetMode::Request::PERFORMANCE) ? "PERFORMANCE" : "ECO";
        RCLCPP_INFO(get_logger(), "Compressor mode set to %s", resp->message.c_str());
      },
      rmw_qos_profile_services_default,
      action_callback_group_
    );

    // Set up blackboard
    rclcpp::NodeOptions bt_node_options;
    bt_node_options.use_global_arguments(false);
    bt_node_ = std::make_shared<rclcpp::Node>(std::string(get_name()) + "_bt", bt_node_options);
    shared_blackboard_ = BT::Blackboard::create();

    // Setup telemetry ingresion
    telemetry_cache_ = std::make_shared<CompressorTelemetryCache>();
    shared_blackboard_->set("telemetry_cache", telemetry_cache_);
    telemetry_sub_ = create_subscription<mserve_interfaces::msg::CompressorTelemetry>(
    "compressor_telemetry", 10,
    [this](std::shared_ptr<const mserve_interfaces::msg::CompressorTelemetry> msg) {
        telemetry_cache_->update(msg, now());
    });

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

  for (auto & slot : ops_) {
    if (slot.tree)  { slot.tree->haltTree(); slot.tree.reset(); }
    if (slot.goal)  { compressor_action_->abort_goal(slot.goal, "<reason>"); slot.goal.reset(); }
  }

  RCLCPP_INFO(get_logger(), "hyfleet_compression deactivated");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressorNode::on_cleanup(const rclcpp_lifecycle::State &) {
  set_tick_timer(false);

  for (auto & slot : ops_) {
      if (slot.tree)  { slot.tree->haltTree(); slot.tree.reset(); }
      if (slot.goal)  { compressor_action_->abort_goal(slot.goal, "<reason>"); slot.goal.reset(); }
  }

  set_mode_srv_.reset();
  telemetry_sub_.reset();
  telemetry_cache_.reset();
  bt_node_.reset();
  shared_blackboard_.reset();
  factory_.reset();

  if (compressor_action_) {
    compressor_action_->unconfigure();
  }

  RCLCPP_INFO(get_logger(), "hyfleet_compression cleaned up");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CompressorNode::on_shutdown(const rclcpp_lifecycle::State &) {
  set_tick_timer(false);

  for (auto & slot : ops_) {
      if (slot.tree)  { slot.tree->haltTree(); slot.tree.reset(); }
      if (slot.goal)  { compressor_action_->abort_goal(slot.goal, "<reason>"); slot.goal.reset(); }
  }

  RCLCPP_INFO(get_logger(), "hyfleet_compression shutdown");
  return CallbackReturn::SUCCESS;
}

// ==============================================================================
// Behavior Tree
// ==============================================================================

void CompressorNode::register_bt_nodes() {
  factory_->registerBuilder<BoostLow>("BoostLow",
    [this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<BoostLow>(name, config, BT::RosNodeParams(bt_node_));
    });
  factory_->registerBuilder<BoostHigh>("BoostHigh",
    [this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<BoostHigh>(name, config, BT::RosNodeParams(bt_node_));
    });
  factory_->registerBuilder<ControlSV>("ControlSV",
  [this](const std::string & name, const BT::NodeConfig & config) {
    return std::make_unique<ControlSV>(name, config, BT::RosNodeParams(bt_node_));
  });
  factory_->registerNodeType<InterstageAboveBand>("InterstageAboveBand");
}

void CompressorNode::build_bt_trees() {
  const std::string base = ament_index_cpp::get_package_share_directory("hyfleet_compressor") + "/trees/";
  factory_->registerBehaviorTreeFromFile(base + "parallel_low.xml");
  factory_->registerBehaviorTreeFromFile(base + "parallel_high.xml");
  factory_->registerBehaviorTreeFromFile(base + "sync.xml");
  factory_->registerBehaviorTreeFromFile(base + "stop_sync.xml");
}

bool CompressorNode::start_op(OpSlot & slot, uint8_t command, uint8_t target, double target_pressure) {
  // Determine tree ID
  std::string tree_id;
  if (target == ControlCompressor::Goal::LOW_BOOSTER)       tree_id = "parallel_low";
  else if (target == ControlCompressor::Goal::HIGH_BOOSTER) tree_id = "parallel_high";
  else if (command != ControlCompressor::Goal::START)        tree_id = "stop_sync";
  else                                                        tree_id = "sync";

  slot.blackboard = BT::Blackboard::create(shared_blackboard_);
  slot.blackboard->enableAutoRemapping(true);
  slot.target = target;
  slot.blackboard->set("low_booster_action",  std::string("/low_booster/control_booster"));
  slot.blackboard->set("high_booster_action", std::string("/high_booster/control_booster"));

  // Translate coordinator command to booster command and write goal fields
  using CB = mserve_interfaces::action::ControlBooster;
  uint8_t booster_cmd = (command == ControlCompressor::Goal::START)   ? CB::Goal::COMPRESS
                      : (command == ControlCompressor::Goal::STOP)    ? CB::Goal::STOP
                                                                      : CB::Goal::FORCE_STOP;
  slot.blackboard->set("command",         booster_cmd);
  slot.blackboard->set("on_target",       uint8_t{CB::Goal::ON_TARGET_SUCCEED});
  slot.blackboard->set("target_pressure", target_pressure);
  slot.blackboard->set("speed_rpm", default_speed_rpm_);
  slot.blackboard->set("cpm", (current_mode_ == SetMode::Request::PERFORMANCE) ? performance_cpm_ : eco_cpm_);

  // Instantiate the tree against this slot's blackboard
  try {
      slot.tree = factory_->createTree(tree_id, slot.blackboard);
  } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "start_op: failed to create tree '%s': %s", tree_id.c_str(), e.what());
      return false;
  }
  return true;
}

void CompressorNode::tick_tree_once() {
    bool any_active = false;
    for (auto & slot : ops_) {
        if (!slot.active()) continue;
        any_active = true;

        BT::NodeStatus status;
        try {
            status = slot.tree->tickOnce();
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "Coordinator tree tick failed: %s", e.what());
            compressor_action_->abort_goal(slot.goal, "coordinator tree tick failed");
            slot.tree->haltTree();
            slot.tree.reset();
            slot.goal.reset();
            continue;
        }

        // Publish feedback from this slot's blackboard
        auto fb = std::make_shared<ControlCompressor::Feedback>();
        double pressure = 0.0, pct = 0.0;
        if (slot.target == ControlCompressor::Goal::LOW_BOOSTER) {
            (void)slot.blackboard->get("low_pressure",          pressure);
            (void)slot.blackboard->get("low_percent_complete",  pct);
        } else {
            // HIGH and SYNC both report the high booster outlet
            (void)slot.blackboard->get("high_pressure",         pressure);
            (void)slot.blackboard->get("high_percent_complete", pct);
        }
        fb->pressure         = pressure;
        fb->percent_complete = pct;
        slot.goal->publish_feedback(fb);

        if (status == BT::NodeStatus::RUNNING) continue;

        // Tree finished — finalise the slot
        if (status == BT::NodeStatus::SUCCESS) {
            compressor_action_->succeed_goal(slot.goal);
        } else {
            compressor_action_->abort_goal(slot.goal, "coordinator tree failed");
        }
        slot.tree->haltTree();
        slot.tree.reset();
        slot.goal.reset();
    }

    if (!any_active) set_tick_timer(false);
}

// ==============================================================================
// Goal and tick
// ==============================================================================

void CompressorNode::on_compressor_goal_accepted( std::shared_ptr<GoalHandleControlCompressor> goal_handle) {
    if (!goal_handle) { return; }

    const auto goal = goal_handle->get_goal();

    // Validate command
    if (goal->command != ControlCompressor::Goal::START && goal->command != ControlCompressor::Goal::STOP && goal->command != ControlCompressor::Goal::FORCE_STOP) {
        compressor_action_->abort_goal(goal_handle, "unknown command");
        return;
    }

    // Validate target
    if (goal->target != ControlCompressor::Goal::LOW_BOOSTER && goal->target != ControlCompressor::Goal::HIGH_BOOSTER && goal->target != ControlCompressor::Goal::SYNC_BOOSTERS) {
        compressor_action_->abort_goal(goal_handle, "unknown target");
        return;
    }

    // Validate pressure (only meaningful for START — STOP/FORCE_STOP carry no target)
    if (goal->command == ControlCompressor::Goal::START &&
        (goal->target_pressure < min_pressure_bar_ || goal->target_pressure > max_pressure_bar_)) {
        compressor_action_->abort_goal(goal_handle, "target_pressure out of range");
        return;
    }

    // SYNC conflicts with everything — abort all active slots
    if (goal->target == ControlCompressor::Goal::SYNC_BOOSTERS) {
        for (auto & s : ops_) {
            if (s.active()) {
                compressor_action_->abort_goal(s.goal, "replaced by SYNC goal");
                s.tree->haltTree();
                s.tree.reset();
                s.goal.reset();
            }
        }
    }

    // Assign slot — LOW→0, HIGH→1, SYNC→0
    const size_t idx = (goal->target == ControlCompressor::Goal::HIGH_BOOSTER) ? 1 : 0;
    auto & slot = ops_[idx];

    // Preempt existing occupant of this slot
    if (slot.active()) {
        compressor_action_->abort_goal(slot.goal, "replaced by newer goal");
        slot.tree->haltTree();
        slot.tree.reset();
        slot.goal.reset();
    }

    if (!start_op(slot, goal->command, goal->target, goal->target_pressure)) {
        compressor_action_->abort_goal(goal_handle, "failed to start operation");
        return;
    }

    slot.goal = std::move(goal_handle);
    set_tick_timer(true);

    RCLCPP_INFO(get_logger(), "Coordinator goal accepted — target=%d command=%d", goal->target, goal->command);
}

void CompressorNode::set_tick_timer(bool enable)
{
  if (tick_timer_) { tick_timer_->cancel(); tick_timer_.reset(); }
  if (enable) {
    tick_timer_ = create_wall_timer(100ms, [this]() { this->tick_tree_once(); }, action_callback_group_);
  }
}

}  // namespace hyfleet_compressor

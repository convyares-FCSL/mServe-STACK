#include <algorithm>
#include <vector>
#include <chrono>

#include <hyfleet_booster/booster_node.hpp>
#include "hyfleet_booster/booster_limits.hpp"
#include "mserve_utils/utils.hpp"
#include "mserve_utils/config.hpp"
#include "include/booster_action.hpp"
#include "include/booster_bt_actions.hpp"
#include "include/booster_bt_conditions.hpp"
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
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
    // Create action booster and configure
    action_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    booster_action_ = std::make_unique<BoosterAction>(*this, "~/control_booster");
    booster_action_->configure(action_callback_group_);
      
    booster_action_->set_goal_callback([this](auto goal_handle) { this->on_booster_goal_accepted(goal_handle); });

    // BT node registration and tree loading
    bt_node_ = std::make_shared<rclcpp::Node>(std::string(get_name()) + "_bt");
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
// Parameters
// ==============================================================================

void BoosterNode::declare_params()
{
  //Hardware mapping
  // VFD index.
  const auto vfd_index_descriptor =  mserve_utils::make_int_range_descriptor("Index into configured VFD mapping[2]", 0, 1);
  declare_parameter<int>("vfd_index", 0, vfd_index_descriptor);

  // Heater index.
  const auto heater_index_descriptor =  mserve_utils::make_int_range_descriptor("Index into configured Heater mapping[2]", 0, 1);
  declare_parameter<int>("heater_index", 0, heater_index_descriptor);

  // Pressure Transducers
  const auto pt_index_descriptor = mserve_utils::make_int_range_descriptor("Index into CompressorTelemetry::pt_bar[16]", 0, 15);
  declare_parameter<int>("inlet_pt_index", 0, pt_index_descriptor);
  declare_parameter<int>("outlet_pt_index", 1, pt_index_descriptor);
  declare_parameter<int>("hyd_primer_pt_index", 2, pt_index_descriptor);
  declare_parameter<int>("hyd_a_pt_index", 3, pt_index_descriptor);
  declare_parameter<int>("hyd_b_pt_index", 4, pt_index_descriptor);
  declare_parameter<int>("coolant_pt_index", 5, pt_index_descriptor);

  // Temperature Transducers
  const auto tt_index_descriptor = mserve_utils::make_int_range_descriptor("Index into CompressorTelemetry::tt_celsius[12]", 0, 11);
  declare_parameter<int>("inlet_tt_index_1", 0, tt_index_descriptor);
  declare_parameter<int>("inlet_tt_index_2", 1, tt_index_descriptor);
  declare_parameter<int>("outlet_tt_index_1", 2, tt_index_descriptor);
  declare_parameter<int>("outlet_tt_index_2", 3, tt_index_descriptor);
  declare_parameter<int>("outlet_tt_index_3", 4, tt_index_descriptor);

  // Position Sensor
  const auto ps_index_descriptor = mserve_utils::make_int_range_descriptor("Index into CompressorTelemetry::ps[4]", 0, 3);
  declare_parameter<int>("ps_lhs_index", 0, ps_index_descriptor);
  declare_parameter<int>("ps_rhs_index", 1, ps_index_descriptor);

  // Solonoid Valves
  declare_parameter<std::string>("inlet_sv_id", "inlet_sv");
  declare_parameter<std::string>("hpu_sv_id", "hpu_sv");

  // Operational parameters
  // Pressure
  const auto pressure_bar_descriptor = mserve_utils::make_double_range_descriptor(
    "Pressure value in bar. Bounded by booster machine system pressure limit.",
    mserve_utils::system_pressure_min_bar, mserve_utils::system_pressure_max_bar);
  declare_parameter<double>("min_pressure_bar", 35.0, pressure_bar_descriptor);
  declare_parameter<double>("max_pressure_bar", 350.0, pressure_bar_descriptor);
  declare_parameter<double>("safe_pressure", 25.0, pressure_bar_descriptor);
  declare_parameter<double>("target_deadband", 0.5, pressure_bar_descriptor);
  declare_parameter<double>("stability_tolerance", 0.05, pressure_bar_descriptor);

  // Temperature
  const auto pt100_temp_descriptor = mserve_utils::make_double_range_descriptor(
      "Temperature value in degC. Valid range: -200..850 degC for Pt100.",
      mserve_utils::pt100_min, mserve_utils::pt100_max);
  declare_parameter<double>("min_temp_inlet", 0.00, pt100_temp_descriptor);
  declare_parameter<double>("max_temp_inlet", 50.0, pt100_temp_descriptor);
  declare_parameter<double>("min_temp_outlet", 0.0, pt100_temp_descriptor);
  declare_parameter<double>("max_temp_outlet", 190.0, pt100_temp_descriptor);

  // Timing
  const auto milliseconds_descriptor = mserve_utils::make_int_range_descriptor(
      "Duration in milliseconds. Valid range: 0..30000 ms.", 
      mserve_utils::timing_min, mserve_utils::timing_max);
  declare_parameter<int>("vfd_delay_ms", 2000, milliseconds_descriptor);
  declare_parameter<int>("vfd_stabilization_ms", 1000, milliseconds_descriptor);

  // VFD Speed
  const auto speed_descriptor = mserve_utils::make_double_range_descriptor(
    "Speed value in rpm", 
    hyfleet_booster::speed_min, hyfleet_booster::speed_max);
  declare_parameter<double>("stop_threshold", 25.0, speed_descriptor);
  declare_parameter<double>("ramp_tolerance", 25.0, speed_descriptor);

  // Count
  const auto sample_count_descriptor =  mserve_utils::make_int_range_descriptor(
    "Number of stabilization samples. Valid range: 1..100.", 1, 100);
  declare_parameter<int>("stabilization_samples", 3, sample_count_descriptor);
}

rcl_interfaces::msg::SetParametersResult BoosterNode::on_parameters(const std::vector<rclcpp::Parameter> & params){
  (void) params;
  
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
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

  
    // Register Conditions
    factory_.registerBuilder<InletPressureStable>("InletPressureStable",
      [this](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<InletPressureStable>(name, config, BT::RosNodeParams(bt_node_));
    });
    factory_.registerBuilder<VFDAtSpeed>("VFDAtSpeed",
      [this](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<VFDAtSpeed>(name, config, BT::RosNodeParams(bt_node_));
    });
    factory_.registerBuilder<OutletAtPressure>("OutletAtPressure",
      [this](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<OutletAtPressure>(name, config, BT::RosNodeParams(bt_node_));
    });
    factory_.registerBuilder<InletPressureSafe>("InletPressureSafe",
      [this](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<InletPressureSafe>(name, config, BT::RosNodeParams(bt_node_));
    });
}

void BoosterNode::build_bt_trees(){
    const std::string base = ament_index_cpp::get_package_share_directory("hyfleet_booster") + "/trees/";
    trees_[0] = factory_.createTreeFromFile(base + "start_tree.xml");
    trees_[1] = factory_.createTreeFromFile(base + "start_idle_tree.xml");
    trees_[2] = factory_.createTreeFromFile(base + "stop_tree.xml");
    trees_[3] = factory_.createTreeFromFile(base + "stop_force_tree.xml");
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

void BoosterNode::on_booster_goal_accepted(std::shared_ptr<GoalHandleControlBooster> goal_handle) {
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "Received null booster goal handle");
    return;
  }

  if (active_goal_) {
    booster_action_->abort_goal(active_goal_, "booster goal replaced by newer goal");
    active_goal_.reset();
  }

  if (active_tree_) { active_tree_->haltTree(); }

  if (!select_tree(goal_handle->get_goal()->command)) {
    RCLCPP_ERROR(get_logger(), "Unknown command: %d", goal_handle->get_goal()->command);
    booster_action_->abort_goal(goal_handle, "unknown command");
    return;
  }

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
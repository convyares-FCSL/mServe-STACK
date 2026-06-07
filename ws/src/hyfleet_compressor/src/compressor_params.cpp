#include <hyfleet_compressor/compressor_node.hpp>
#include "hyfleet_compressor/compressor_limits.hpp"
#include "hyfleet_booster/booster_limits.hpp"
#include "mserve_utils/utils.hpp"
#include "mserve_utils/config.hpp"
#include <lifecycle_msgs/msg/state.hpp>

// ==============================================================================
// Parameter Handling for Compressor
// ==============================================================================

namespace hyfleet_compressor {

// Declares all booster paramets  
void CompressorNode::declare_params()
{
  //Hardware mapping

  // Pressure Transducers
  const auto pt_index_descriptor = mserve_utils::make_int_range_descriptor("Index into CompressorTelemetry::pt_bar[16]", 0, 15);
  declare_parameter<int>("inlet_pt_index", 0, pt_index_descriptor);
  declare_parameter<int>("interstage_pt_index", 1, pt_index_descriptor);
  declare_parameter<int>("outlet_pt_index", 2, pt_index_descriptor);

  // Solenoid Valves
  const auto sv_index_descriptor = mserve_utils::make_int_range_descriptor("Index into CompressorTelemetry::sv[5]", 0, 4);
  declare_parameter<int>("interstage_sv_index", 0, sv_index_descriptor);

  // Operational — goal validation
  // Pressure
  const auto pressure_bar_descriptor = mserve_utils::make_double_range_descriptor(
    "Pressure value in bar. Bounded by system pressure limit.",
    mserve_utils::system_pressure_min_bar, mserve_utils::system_pressure_max_bar);
  declare_parameter<double>("min_pressure_bar", 35.0,  pressure_bar_descriptor);
  declare_parameter<double>("max_pressure_bar", 700.0, pressure_bar_descriptor);

  // Speed
  const auto speed_rpm_descriptor = mserve_utils::make_double_range_descriptor(
    "Speed in rpm. Bounded by booster speed limit.",
    hyfleet_booster::speed_min, hyfleet_booster::speed_max);
  declare_parameter<double>("default_speed_rpm", 1500.0, speed_rpm_descriptor);

  // CPM
  const auto cpm_descriptor = mserve_utils::make_double_range_descriptor(
    "Cycles per minute. Bounded by booster cpm limit.",
    hyfleet_booster::cpm_min, hyfleet_booster::cpm_max);
  declare_parameter<double>("performance_cpm", 16.0, cpm_descriptor);
  declare_parameter<double>("eco_cpm", 8.0, cpm_descriptor);
}

// Loads all parameter and updates blackboard
void CompressorNode::load_params()
{
  auto & p = *get_node_parameters_interface();

   // Architecture contracts (hardcoded)
  shared_blackboard_ ->set("service_name", std::string("/") + get_name() + "/compressor_cmd");
  
  // Architecture contracts (hardcoded)
  shared_blackboard_ ->set("low_booster_action",  std::string("/low_booster/control_booster"));
  shared_blackboard_ ->set("high_booster_action", std::string("/high_booster/control_booster"));


  // Hardware mapping (blackboard only)
  // Pressure Transudcer
  const int inlet_pt_index = mserve_utils::get_or_declare_param(p, get_logger(), "inlet_pt_index", 0, "Inlet H2 PT index");
  shared_blackboard_ ->set("inlet_pt_index", inlet_pt_index);
  const int interstage_pt_index = mserve_utils::get_or_declare_param(p, get_logger(), "interstage_pt_index", 1, "Interstage H2 PT index");
  shared_blackboard_ ->set("interstage_pt_index", interstage_pt_index);
  const int outlet_pt_index = mserve_utils::get_or_declare_param(p, get_logger(), "outlet_pt_index", 2, "Outlet H2 PT index");
  shared_blackboard_ ->set("outlet_pt_index", outlet_pt_index);

  // Solenoid Valves
  const int interstage_sv_index = mserve_utils::get_or_declare_param(p, get_logger(), "interstage_sv_index", 0, "Interstage SV index");
  shared_blackboard_ ->set("interstage_sv_index", interstage_sv_index);

  // Goal validation limits — also stored as members for use in on_compressor_goal_accepted
  min_pressure_bar_ = mserve_utils::get_or_declare_param(p, get_logger(), "min_pressure_bar", 35.0,  "Minimum valid goal pressure (bar)");
  shared_blackboard_ ->set("min_pressure_bar", min_pressure_bar_);
  max_pressure_bar_ = mserve_utils::get_or_declare_param(p, get_logger(), "max_pressure_bar", 900.0, "Maximum valid goal pressure (bar)");
  shared_blackboard_ ->set("max_pressure_bar", max_pressure_bar_);

  // VFD Speed
  default_speed_rpm_ = mserve_utils::get_or_declare_param(p, get_logger(), "default_speed_rpm", 1500.0, "VFD target speed (rpm)");
  shared_blackboard_->set("default_speed_rpm", default_speed_rpm_);

  // Cycles per minute 
  performance_cpm_ = mserve_utils::get_or_declare_param(p, get_logger(), "performance_cpm", 16.0, "PCSV rate in PERFORMANCE mode");
  shared_blackboard_->set("performance_cpm", performance_cpm_);
  eco_cpm_ = mserve_utils::get_or_declare_param(p, get_logger(), "eco_cpm", 8.0, "PCSV rate in ECO mode");
  shared_blackboard_->set("eco_cpm", eco_cpm_);

}

rcl_interfaces::msg::SetParametersResult CompressorNode::on_parameters(const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    result.successful = false;
    result.reason = "Parameters can only be changed in UNCONFIGURED state";
    return result;
  }

  // Helper lamda for checking cross parameter
  auto effective = [&](const std::string & name, double current) -> double {
    for (const auto & p : params) {
      if (p.get_name() == name) return p.as_double();
    }
    return current;
  };

  // Check pressure parameter
  const double new_min = effective("min_pressure_bar", get_parameter("min_pressure_bar").as_double());
  const double new_max = effective("max_pressure_bar", get_parameter("max_pressure_bar").as_double());
  if (new_min >= new_max) {
    result.successful = false;
    result.reason = "min_pressure_bar must be less than max_pressure_bar";
    return result;
  }

  result.successful = true;
  return result;
}

}  // namespace hyfleet_compressor

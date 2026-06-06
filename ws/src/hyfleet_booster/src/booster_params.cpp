#include <hyfleet_booster/booster_node.hpp>
#include "hyfleet_booster/booster_limits.hpp"
#include "mserve_utils/utils.hpp"
#include "mserve_utils/config.hpp"
#include <lifecycle_msgs/msg/state.hpp>

// ==============================================================================
// Parameter Handling for Booster
// ==============================================================================

namespace hyfleet_booster {

// Declares all booster paramets  
void BoosterNode::declare_params(){
  //Hardware mapping
  // VFD index.
  const auto vfd_index_descriptor =  mserve_utils::make_int_range_descriptor("Index into configured VFD mapping[2]", 0, 1);
  declare_parameter<int>("vfd_index", 0, vfd_index_descriptor);

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

  // Solenoid Valves
  const auto sv_index_descriptor = mserve_utils::make_int_range_descriptor("Index into CompressorTelemetry::sv[5]", 0, 4);
  declare_parameter<int>("inlet_sv_index", 0, sv_index_descriptor);
  declare_parameter<int>("hpu_sv_index", 1, sv_index_descriptor);

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
  declare_parameter<int>("stability_timeout_ms", 10000, milliseconds_descriptor);
  declare_parameter<int>("ramp_timeout_ms", 30000, milliseconds_descriptor);
  declare_parameter<int>("stop_timeout_ms", 15000, milliseconds_descriptor);

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

// Loads all parameter and updates blackboard
void BoosterNode::load_params(){
  auto & p = *get_node_parameters_interface();
  
  // Architecture contracts (hardcoded)
  blackboard_->set("service_name", std::string("/") + get_name() + "/booster_cmd");

  // Hardware mapping (blackboard only)
  // VFD
  const int vfd_index = mserve_utils::get_or_declare_param(p, get_logger(), "vfd_index", 0, "VFD index");
  blackboard_->set("vfd_index", vfd_index);

  // Pressure Transudcer
  const int inlet_pt_index = mserve_utils::get_or_declare_param(p, get_logger(), "inlet_pt_index", 0, "Inlet H2 PT index");
  blackboard_->set("inlet_pt_index", inlet_pt_index);
  const int outlet_pt_index = mserve_utils::get_or_declare_param(p, get_logger(), "outlet_pt_index", 1, "Outlet H2 PT index");
  blackboard_->set("outlet_pt_index", outlet_pt_index);
  const int hyd_primer_pt_index = mserve_utils::get_or_declare_param(p, get_logger(), "hyd_primer_pt_index", 2, "Hydrollic Primer PT index");
  blackboard_->set("hyd_primer_pt_index", hyd_primer_pt_index);
  const int hyd_a_pt_index = mserve_utils::get_or_declare_param(p, get_logger(), "hyd_a_pt_index", 3, "Hydrollic A Line PT index");
  blackboard_->set("hyd_a_pt_index", hyd_a_pt_index);
  const int hyd_b_pt_index = mserve_utils::get_or_declare_param(p, get_logger(), "hyd_b_pt_index", 4, "Hydrollic B Line PT index");
  blackboard_->set("hyd_b_pt_index", hyd_b_pt_index);
  const int coolant_pt_index = mserve_utils::get_or_declare_param(p, get_logger(), "coolant_pt_index", 5, "Coolant PT index");
  blackboard_->set("coolant_pt_index", coolant_pt_index);

  // Temperature Transudcer
  const int inlet_tt_index_1 = mserve_utils::get_or_declare_param(p, get_logger(), "inlet_tt_index_1", 0, "H2 Booster Inlet A TT index");
  blackboard_->set("inlet_tt_index_1", inlet_tt_index_1);
  const int inlet_tt_index_2 = mserve_utils::get_or_declare_param(p, get_logger(), "inlet_tt_index_2", 1, "H2 Booster Inlet B TT index");
  blackboard_->set("inlet_tt_index_2", inlet_tt_index_2);
  const int outlet_tt_index_1 = mserve_utils::get_or_declare_param(p, get_logger(), "outlet_tt_index_1", 2, "H2 Booster Outlet A TT index");
  blackboard_->set("outlet_tt_index_1", outlet_tt_index_1);
  const int outlet_tt_index_2 = mserve_utils::get_or_declare_param(p, get_logger(), "outlet_tt_index_2", 3, "H2 Booster Outlet B TT index");
  blackboard_->set("outlet_tt_index_2", outlet_tt_index_2);
  const int outlet_tt_index_3 = mserve_utils::get_or_declare_param(p, get_logger(), "outlet_tt_index_3", 4, "H2 Booster Outlet Post Coolant TT index");
  blackboard_->set("outlet_tt_index_3", outlet_tt_index_3);

  // Position Sensor
  const int ps_lhs_index = mserve_utils::get_or_declare_param(p, get_logger(), "ps_lhs_index", 0, "H2 Booster LHS PS index");
  blackboard_->set("ps_lhs_index", ps_lhs_index);
  const int ps_rhs_index = mserve_utils::get_or_declare_param(p, get_logger(), "ps_rhs_index", 1, "H2 Booster RHS PS index");
  blackboard_->set("ps_rhs_index", ps_rhs_index);

  // Solenoid Valves
  const int inlet_sv_index = mserve_utils::get_or_declare_param(p, get_logger(), "inlet_sv_index", 0, "Inlet SV index");
  blackboard_->set("inlet_sv_index", inlet_sv_index);
  const int hpu_sv_index = mserve_utils::get_or_declare_param(p, get_logger(), "hpu_sv_index", 1, "HPU SV index");
  blackboard_->set("hpu_sv_index", hpu_sv_index);

  // --- Operational → member (goal validation) + blackboard (BT nodes) ---
  min_pressure_bar_ = mserve_utils::get_or_declare_param(p, get_logger(), "min_pressure_bar", 35.0, "min pressure");
  blackboard_->set("min_pressure_bar", min_pressure_bar_);
  max_pressure_bar_ = mserve_utils::get_or_declare_param(p, get_logger(), "max_pressure_bar", 350.0, "max pressure");
  blackboard_->set("max_pressure_bar", max_pressure_bar_);
  const double safe_pressure = mserve_utils::get_or_declare_param(p, get_logger(), "safe_pressure", 25.0, "Minimum inlet pressure to run safely (bar)");
  blackboard_->set("safe_pressure", safe_pressure);
  const double target_deadband = mserve_utils::get_or_declare_param(p, get_logger(), "target_deadband", 0.5, "Pressure deadband around target (bar) ");
  blackboard_->set("target_deadband", target_deadband);
  const double stability_tolerance = mserve_utils::get_or_declare_param(p, get_logger(), "stability_tolerance", 0.05, "Max variation for inlet stability check (bar)");
  blackboard_->set("stability_tolerance", stability_tolerance);

  // Temperature
  const double min_temp_inlet = mserve_utils::get_or_declare_param(p, get_logger(), "min_temp_inlet", 0.00, "Minimum inlet temperature (°C)");
  blackboard_->set("min_temp_inlet", min_temp_inlet);
  const double max_temp_inlet = mserve_utils::get_or_declare_param(p, get_logger(), "max_temp_inlet", 50.0, "Maximum inlet temperature (°C) ");
  blackboard_->set("max_temp_inlet", max_temp_inlet);
  const double min_temp_outlet = mserve_utils::get_or_declare_param(p, get_logger(), "min_temp_outlet", 0.0, "Minimum outlet temperature (°C)");
  blackboard_->set("min_temp_outlet", min_temp_outlet);
  const double max_temp_outlet = mserve_utils::get_or_declare_param(p, get_logger(), "max_temp_outlet", 190.0, "Maximum outlet temperature (°C)");
  blackboard_->set("max_temp_outlet", max_temp_outlet);

  // Timing: params are ms, BT Sleep node takes seconds
  const int vfd_delay_ms = mserve_utils::get_or_declare_param(p, get_logger(), "vfd_delay_ms", 2000, "Wait after VFD start before checking speed (ms)");
  blackboard_->set("vfd_delay_ms", vfd_delay_ms);
  const int vfd_stabilization_ms = mserve_utils::get_or_declare_param(p, get_logger(), "vfd_stabilization_ms", 1000, "Wait after VFD at speed before opening HPU (ms)");
  blackboard_->set("vfd_stabilization_ms", vfd_stabilization_ms);
  const int stability_timeout_ms = mserve_utils::get_or_declare_param(p, get_logger(), "stability_timeout_ms", 10000, "Max time for inlet pressure to stabilise before startup (ms)");
  blackboard_->set("stability_timeout_ms", stability_timeout_ms);
  const int ramp_timeout_ms = mserve_utils::get_or_declare_param(p, get_logger(), "ramp_timeout_ms", 30000, "Max time for VFD to reach target speed (ms)");
  blackboard_->set("ramp_timeout_ms", ramp_timeout_ms);
  const int stop_timeout_ms = mserve_utils::get_or_declare_param(p, get_logger(), "stop_timeout_ms", 15000, "Max time for VFD to reach stopped state (ms)");
  blackboard_->set("stop_timeout_ms", stop_timeout_ms);

  // VFD Speed
  const double stop_threshold = mserve_utils::get_or_declare_param(p, get_logger(), "stop_threshold", 25.0, "Band around target — VFD considered at speed (rpm)");
  blackboard_->set("stop_threshold", stop_threshold);
  const double ramp_tolerance = mserve_utils::get_or_declare_param(p, get_logger(), "ramp_tolerance", 25.0, "Below this — VFD considered stopped (rpm)");
  blackboard_->set("ramp_tolerance", ramp_tolerance);

  // Count
  const int stabilization_samples = mserve_utils::get_or_declare_param(p, get_logger(), "stabilization_samples", 3, "Rolling window depth for `InletPressureStable");
  blackboard_->set("stabilization_samples", stabilization_samples);
}

rcl_interfaces::msg::SetParametersResult BoosterNode::on_parameters(const std::vector<rclcpp::Parameter> & params){
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
  const double new_min_pressure = effective("min_pressure_bar", get_parameter("min_pressure_bar").as_double());
  const double new_max_pressure = effective("max_pressure_bar", get_parameter("max_pressure_bar").as_double());
  if (new_min_pressure >= new_max_pressure) {
    result.successful = false;
    result.reason = "min_pressure_bar must be less than max_pressure_bar";
    return result;
  }

  // Check inlet temperature parameter
  const double new_min_temp_inlet  = effective("min_temp_inlet", get_parameter("min_temp_inlet").as_double());
  const double new_max_temp_inlet  = effective("max_temp_inlet", get_parameter("max_temp_inlet").as_double());
  if (new_min_temp_inlet >= new_max_temp_inlet) {
    result.successful = false;
    result.reason = "min_temp_inlet must be less than max_temp_inlet";
    return result;
  }
  
  // Check outlet temperature parameter
  const double new_min_temp_outlet = effective("min_temp_outlet", get_parameter("min_temp_outlet").as_double());
  const double new_max_temp_outlet = effective("max_temp_outlet", get_parameter("max_temp_outlet").as_double());
  if (new_min_temp_outlet >= new_max_temp_outlet) {
    result.successful = false;
    result.reason = "min_temp_outlet must be less than max_temp_outlet";
    return result;
  }

  result.successful = true;
  return result;
}

}  // namespace hyfleet_booster
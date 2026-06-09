#include <hyfleet_subsystem/subsystem_node.hpp>
#include "hyfleet_subsystem/subsystem_limits.hpp"
#include "mserve_utils/utils.hpp"
#include "mserve_utils/config.hpp"
#include <lifecycle_msgs/msg/state.hpp>

// TODO: rename namespace to hyfleet_<name>
namespace hyfleet_subsystem {

void SubsystemNode::declare_params()
{
  // ==============================================================================
  // TODO: declare all subsystem params here with descriptor ranges.
  //
  // Pattern:
  //   const auto descriptor = mserve_utils::make_<type>_range_descriptor("description", min, max);
  //   declare_parameter<T>("param_name", default_value, descriptor);
  //
  // Descriptor helpers:
  //   mserve_utils::make_int_range_descriptor(description, min, max)
  //   mserve_utils::make_double_range_descriptor(description, min, max)
  //
  // Shared descriptor bounds (from mserve_utils/config.hpp):
  //   mserve_utils::system_pressure_min_bar / system_pressure_max_bar
  //   mserve_utils::pt100_min / pt100_max
  //   mserve_utils::timing_min / timing_max
  //
  // Hardware constexpr limits live in subsystem_limits.hpp.
  //
  // Classification rule — differs per instance → param; universal + immutable → constexpr.
  // ==============================================================================

  // Example — hardware mapping indices:
  // const auto pt_index_descriptor = mserve_utils::make_int_range_descriptor(
  //   "Index into CompressorTelemetry::pt_bar[16]", 0, 15);
  // declare_parameter<int>("inlet_pt_index", 0, pt_index_descriptor);

  // Example — operational:
  // const auto pressure_descriptor = mserve_utils::make_double_range_descriptor(
  //   "Pressure in bar.", mserve_utils::system_pressure_min_bar, mserve_utils::system_pressure_max_bar);
  // declare_parameter<double>("min_pressure_bar", 35.0, pressure_descriptor);
  // declare_parameter<double>("max_pressure_bar", 350.0, pressure_descriptor);

  // Example — timing:
  // const auto ms_descriptor = mserve_utils::make_int_range_descriptor(
  //   "Duration in milliseconds.", mserve_utils::timing_min, mserve_utils::timing_max);
  // declare_parameter<int>("timeout_ms", 10000, ms_descriptor);
}

void SubsystemNode::load_params()
{
  auto & p = *get_node_parameters_interface();

  // ==============================================================================
  // Architecture contracts (hardcoded — not params). Write to blackboard here.
  // ==============================================================================

  // TODO: set subsystem service/action name contracts, e.g.:
  // blackboard_->set("service_name", std::string("/") + get_name() + "/<name>_cmd");

  // ==============================================================================
  // Hardware mapping — read params, write to blackboard for BT nodes.
  // ==============================================================================

  // TODO: uncomment and fill in, e.g.:
  // const int inlet_pt_index = mserve_utils::get_or_declare_param(
  //   p, get_logger(), "inlet_pt_index", 0, "Inlet PT index");
  // blackboard_->set("inlet_pt_index", inlet_pt_index);

  // ==============================================================================
  // Operational — read params, store members used for goal validation.
  // ==============================================================================

  // TODO: uncomment and fill in, e.g.:
  // min_pressure_bar_ = mserve_utils::get_or_declare_param(
  //   p, get_logger(), "min_pressure_bar", 35.0, "Minimum goal pressure (bar)");
  // blackboard_->set("min_pressure_bar", min_pressure_bar_);
}

rcl_interfaces::msg::SetParametersResult
SubsystemNode::on_parameters(const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    result.successful = false;
    result.reason = "Parameters can only be changed in UNCONFIGURED state";
    return result;
  }

  // TODO: add cross-parameter validation if needed, e.g. min < max checks.
  // Helper lambda:
  auto effective = [&](const std::string & name, double current) -> double {
    for (const auto & p : params) {
      if (p.get_name() == name) return p.as_double();
    }
    return current;
  };
  (void)effective;  // suppress unused warning until cross-checks are added

  result.successful = true;
  return result;
}

}  // namespace hyfleet_subsystem

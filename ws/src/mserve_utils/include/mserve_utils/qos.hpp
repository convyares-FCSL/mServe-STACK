#ifndef MSERVE_UTILS_QOS_HPP
#define MSERVE_UTILS_QOS_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "mserve_utils/utils.hpp"

// Named QoS profiles for mServe traffic types.
// Each profile reads its settings from parameters so they can be overridden
// in launch config without recompiling.
//
// Profile defaults:
//   commands  — reliable, volatile, depth=1   (only the latest command matters)
//   feedback  — best_effort, volatile, depth=10 (high-rate sensor data, drop ok)
//   status    — reliable, volatile, depth=10  (connection/state changes matter)

namespace mserve_qos {

struct ProfileDefaults {
  std::string reliability;
  std::string durability;
  int depth;
};

inline ProfileDefaults defaults_for(const std::string & profile)
{
  if (profile == "commands") { return {"reliable",    "volatile", 1};  }
  if (profile == "feedback") { return {"best_effort", "volatile", 10}; }
  if (profile == "status")   { return {"reliable",    "volatile", 10}; }
  return {"reliable", "volatile", 10};
}

inline rclcpp::QoS load(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger,
  const std::string & profile)
{
  const auto defs = defaults_for(profile);
  const std::string base = "qos." + profile;

  const auto rel = mserve_utils::get_or_declare_param(
    params, logger, base + ".reliability", defs.reliability, "qos");
  const auto dur = mserve_utils::get_or_declare_param(
    params, logger, base + ".durability",  defs.durability,  "qos");
  const int depth = static_cast<int>(mserve_utils::get_or_declare_param(
    params, logger, base + ".depth", static_cast<double>(defs.depth), "qos"));

  rclcpp::QoS qos(depth);
  if (rel == "best_effort") { qos.best_effort(); } else { qos.reliable(); }
  if (dur == "transient_local") { qos.transient_local(); } else { qos.durability_volatile(); }
  return qos;
}

inline rclcpp::QoS commands(rclcpp_lifecycle::LifecycleNode & node)
{
  return load(*node.get_node_parameters_interface(), node.get_logger(), "commands");
}

inline rclcpp::QoS feedback(rclcpp_lifecycle::LifecycleNode & node)
{
  return load(*node.get_node_parameters_interface(), node.get_logger(), "feedback");
}

inline rclcpp::QoS status(rclcpp_lifecycle::LifecycleNode & node)
{
  return load(*node.get_node_parameters_interface(), node.get_logger(), "status");
}

}  // namespace mserve_qos

#endif  // MSERVE_UTILS_QOS_HPP

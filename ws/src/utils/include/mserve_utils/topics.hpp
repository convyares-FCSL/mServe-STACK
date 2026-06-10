#ifndef MSERVE_UTILS_TOPICS_HPP
#define MSERVE_UTILS_TOPICS_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "mserve_utils/utils.hpp"

// mServe canonical topic names, backed by parameters so they can be remapped
// in launch config without recompiling. All mServe-internal topics use the
// /mserve/ namespace. /cmd_vel stays at the ROS standard path so Nav2 and
// joystick drivers work without remapping.

namespace mserve_topics {

inline std::string cmd_vel(rclcpp_lifecycle::LifecycleNode & node)
{
  return mserve_utils::get_or_declare_param(
    *node.get_node_parameters_interface(), node.get_logger(),
    "topic_names.cmd_vel", std::string("/cmd_vel"), "topic");
}

inline std::string cmd_vel_safe(rclcpp_lifecycle::LifecycleNode & node)
{
  return mserve_utils::get_or_declare_param(
    *node.get_node_parameters_interface(), node.get_logger(),
    "topic_names.cmd_vel_safe", std::string("/mserve/cmd_vel_safe"), "topic");
}

inline std::string wheel_feedback(rclcpp_lifecycle::LifecycleNode & node)
{
  return mserve_utils::get_or_declare_param(
    *node.get_node_parameters_interface(), node.get_logger(),
    "topic_names.wheel_feedback", std::string("/mserve/wheel_feedback"), "topic");
}

inline std::string drivechain_status(rclcpp_lifecycle::LifecycleNode & node)
{
  return mserve_utils::get_or_declare_param(
    *node.get_node_parameters_interface(), node.get_logger(),
    "topic_names.drivechain_status", std::string("/mserve/drivechain_status"), "topic");
}

}  // namespace mserve_topics

#endif  // MSERVE_UTILS_TOPICS_HPP

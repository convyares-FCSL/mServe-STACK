#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

#include "lifecycle_msgs/msg/transition.hpp"

namespace mserve_utils
{
namespace lifecycle
{

inline std::optional<uint8_t> transitionIdFromName(const std::string& transition_name)
{
  static const std::unordered_map<std::string, uint8_t> transitions = {
    {"configure", lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE},
    {"activate", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE},
    {"deactivate", lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE},
    {"cleanup", lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP},

    {"shutdown_unconfigured", lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN},
    {"shutdown_inactive", lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN},
    {"shutdown_active", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN},

    {"destroy", lifecycle_msgs::msg::Transition::TRANSITION_DESTROY},
  };

  const auto it = transitions.find(transition_name);
  if (it == transitions.end()) {
    return std::nullopt;
  }

  return it->second;
}

}  // namespace lifecycle
}  // namespace mserve_utils
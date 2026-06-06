#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "interfaces/srv/booster_cmd.hpp"
#include "interfaces/srv/compressor_cmd.hpp"

#include "internal/device_types.hpp"

namespace compressor
{

class DeviceDispatcher
{
public:
  DeviceDispatcher(
    rclcpp_lifecycle::LifecycleNode & node,
    DeviceDispatcherConfig config,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr);

  bool dispatch(const DeviceCommands & commands);

private:
  struct RequestContext
  {
    std::string device_id;
    std::string device_name;
    std::string service_group;
  };

  bool send_booster_request(
    const rclcpp::Client<interfaces::srv::BoosterCmd>::SharedPtr & client,
    const std::string & service_name,
    int32_t booster_id,
    uint8_t command,
    const std::array<int32_t, 5> & payload,
    RequestContext context);
  bool send_compressor_request(
    uint8_t command,
    const std::array<int32_t, 5> & payload,
    RequestContext context);

  rclcpp_lifecycle::LifecycleNode & node_;
  DeviceDispatcherConfig config_;
  rclcpp::Client<interfaces::srv::BoosterCmd>::SharedPtr control_low_booster_client_;
  rclcpp::Client<interfaces::srv::BoosterCmd>::SharedPtr control_high_booster_client_;
  rclcpp::Client<interfaces::srv::CompressorCmd>::SharedPtr control_compressor_client_;
};

}  // namespace compressor

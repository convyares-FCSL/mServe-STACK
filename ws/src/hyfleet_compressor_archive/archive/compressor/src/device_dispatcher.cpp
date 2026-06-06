#include "internal/device_dispatcher.hpp"

#include "interfaces/msg/booster_cmd.hpp"
#include "interfaces/msg/compressor_cmd.hpp"
#include "interfaces/msg/h_result.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <future>
#include <utility>

namespace compressor
{

namespace
{
bool command_succeeded(const interfaces::msg::HResult & result)
{
  return result.value == interfaces::msg::HResult::S_OK;
}

bool validate_pcsv(
  const PCSV & pcsv,
  const DeviceLimits & limits,
  const rclcpp::Logger & logger)
{
  if (pcsv.enable &&
    (!std::isfinite(pcsv.cpm) ||
    pcsv.cpm < limits.pcsv_min_cpm ||
    pcsv.cpm > limits.pcsv_max_cpm))
  {
    RCLCPP_ERROR(
      logger,
      "Refusing PCSV command for %s: cpm %.2f outside [%.2f, %.2f]",
      pcsv.device_id.c_str(),
      pcsv.cpm,
      limits.pcsv_min_cpm,
      limits.pcsv_max_cpm);
    return false;
  }

  if (!pcsv.enable && pcsv.cpm != 0.0) {
    RCLCPP_ERROR(
      logger,
      "Refusing PCSV off command for %s: cpm must be 0.0, got %.2f",
      pcsv.device_id.c_str(),
      pcsv.cpm);
    return false;
  }

  return true;
}

bool validate_vfd(
  const VFD & vfd,
  const DeviceLimits & limits,
  const rclcpp::Logger & logger)
{
  if (!std::isfinite(vfd.speed_rpm)) {
    RCLCPP_ERROR(
      logger,
      "Refusing VFD command for %s: rpm must be finite",
      vfd.device_id.c_str());
    return false;
  }

  if (!vfd.on_off && vfd.speed_rpm != 0.0) {
    RCLCPP_ERROR(
      logger,
      "Refusing VFD off command for %s: speed must be 0.0, got %.2f",
      vfd.device_id.c_str(),
      vfd.speed_rpm);
    return false;
  }

  if (vfd.on_off &&
    (vfd.speed_rpm < limits.vfd_min_speed_rpm ||
    vfd.speed_rpm > limits.vfd_max_speed_rpm))
  {
    RCLCPP_ERROR(
      logger,
      "Refusing VFD command for %s: rpm %.2f outside [%.2f, %.2f]",
      vfd.device_id.c_str(),
      vfd.speed_rpm,
      limits.vfd_min_speed_rpm,
      limits.vfd_max_speed_rpm);
    return false;
  }

  return true;
}

bool validate_heater(
  const Heater & heater,
  const std::vector<uint8_t> & valid_heater_ids,
  const rclcpp::Logger & logger)
{
  if (std::find(valid_heater_ids.begin(), valid_heater_ids.end(), heater.heater_id) ==
    valid_heater_ids.end())
  {
    RCLCPP_ERROR(
      logger,
      "Refusing unknown heater target: %u",
      heater.heater_id);
    return false;
  }
  return true;
}
}  // namespace

DeviceDispatcher::DeviceDispatcher(
  rclcpp_lifecycle::LifecycleNode & node,
  DeviceDispatcherConfig config,
  rclcpp::CallbackGroup::SharedPtr callback_group)
: node_(node), config_(std::move(config))
{
  control_low_booster_client_ = node_.create_client<interfaces::srv::BoosterCmd>(
    config_.services.control_low_booster_service_name,
    rclcpp::ServicesQoS(),
    callback_group);
  control_high_booster_client_ = node_.create_client<interfaces::srv::BoosterCmd>(
    config_.services.control_high_booster_service_name,
    rclcpp::ServicesQoS(),
    callback_group);
  control_compressor_client_ = node_.create_client<interfaces::srv::CompressorCmd>(
    config_.services.control_compressor_service_name,
    rclcpp::ServicesQoS(),
    callback_group);
}

bool DeviceDispatcher::dispatch(const DeviceCommands & commands)
{
  const auto logger = node_.get_logger();
  const auto & low_service = config_.services.control_low_booster_service_name;
  const auto & high_service = config_.services.control_high_booster_service_name;
  bool all_acknowledged = true;

  for (const auto & vfd : commands.vfd) {
    const uint8_t command = vfd.enable && vfd.on_off ?
      interfaces::msg::BoosterCmd::START_VFD :
      interfaces::msg::BoosterCmd::STOP_VFD;

    if (!validate_vfd(vfd, config_.limits, logger)) {
      all_acknowledged = false;
      continue;
    }

    std::array<int32_t, 5> payload{};
    payload[0] = static_cast<int32_t>(std::lround(vfd.speed_rpm));

    if (vfd.device_id == config_.low_booster.vfd_id) {
      all_acknowledged = send_booster_request(
        control_low_booster_client_,
        low_service,
        config_.command_ids.low_booster,
        command,
        payload,
        {vfd.device_id, "VFD", "booster"}) && all_acknowledged;
    } else if (vfd.device_id == config_.high_booster.vfd_id) {
      all_acknowledged = send_booster_request(
        control_high_booster_client_,
        high_service,
        config_.command_ids.high_booster,
        command,
        payload,
        {vfd.device_id, "VFD", "booster"}) && all_acknowledged;
    } else {
      RCLCPP_ERROR(logger, "Refusing unknown VFD target: %s", vfd.device_id.c_str());
      all_acknowledged = false;
    }
  }

  for (const auto & pcsv : commands.pcsv) {
    if (!validate_pcsv(pcsv, config_.limits, logger)) {
      all_acknowledged = false;
      continue;
    }

    std::array<int32_t, 5> payload{};
    payload[0] = static_cast<int32_t>(pcsv.enable ? 1 : 0);
    payload[1] = static_cast<int32_t>(std::lround(pcsv.cpm));

    if (pcsv.device_id == config_.low_booster.pcsv_id) {
      all_acknowledged = send_booster_request(
        control_low_booster_client_,
        low_service,
        config_.command_ids.low_booster,
        interfaces::msg::BoosterCmd::SET_PCSV,
        payload,
        {pcsv.device_id, "PCSV", "booster"}) && all_acknowledged;
    } else if (pcsv.device_id == config_.high_booster.pcsv_id) {
      all_acknowledged = send_booster_request(
        control_high_booster_client_,
        high_service,
        config_.command_ids.high_booster,
        interfaces::msg::BoosterCmd::SET_PCSV,
        payload,
        {pcsv.device_id, "PCSV", "booster"}) && all_acknowledged;
    } else {
      RCLCPP_ERROR(logger, "Refusing unknown PCSV target: %s", pcsv.device_id.c_str());
      all_acknowledged = false;
    }
  }

  for (const auto & sv : commands.sv) {
    std::array<int32_t, 5> payload{};
    payload[0] = static_cast<int32_t>(sv.state ? 1 : 0);

    if (sv.device_id == config_.low_booster.inlet_sv_id) {
      all_acknowledged = send_booster_request(
        control_low_booster_client_,
        low_service,
        config_.command_ids.low_booster,
        interfaces::msg::BoosterCmd::CONTROL_INLET_SV,
        payload,
        {sv.device_id, "SV", "booster"}) && all_acknowledged;
    } else if (sv.device_id == config_.low_booster.hpu_sv_id) {
      all_acknowledged = send_booster_request(
        control_low_booster_client_,
        low_service,
        config_.command_ids.low_booster,
        interfaces::msg::BoosterCmd::CONTROL_HPU_SV,
        payload,
        {sv.device_id, "SV", "booster"}) && all_acknowledged;
    } else if (sv.device_id == config_.high_booster.inlet_sv_id) {
      all_acknowledged = send_booster_request(
        control_high_booster_client_,
        high_service,
        config_.command_ids.high_booster,
        interfaces::msg::BoosterCmd::CONTROL_INLET_SV,
        payload,
        {sv.device_id, "SV", "booster"}) && all_acknowledged;
    } else if (sv.device_id == config_.high_booster.hpu_sv_id) {
      all_acknowledged = send_booster_request(
        control_high_booster_client_,
        high_service,
        config_.command_ids.high_booster,
        interfaces::msg::BoosterCmd::CONTROL_HPU_SV,
        payload,
        {sv.device_id, "SV", "booster"}) && all_acknowledged;
    } else {
      RCLCPP_ERROR(logger, "Refusing unknown SV target: %s", sv.device_id.c_str());
      all_acknowledged = false;
    }
  }

  for (const auto & heater : commands.heater) {
    if (!validate_heater(heater, config_.command_ids.heater_ids, logger)) {
      all_acknowledged = false;
      continue;
    }

    std::array<int32_t, 5> payload{};
    payload[0] = heater.heater_id;
    payload[1] = heater.state ? 1 : 0;

    all_acknowledged = send_compressor_request(
      interfaces::msg::CompressorCmd::CONTROL_HEATER,
      payload,
      {"heater " + std::to_string(heater.heater_id), "heater", "compressor"}) &&
      all_acknowledged;
  }

  return all_acknowledged;
}

bool DeviceDispatcher::send_booster_request(
  const rclcpp::Client<interfaces::srv::BoosterCmd>::SharedPtr & client,
  const std::string & service_name,
  int32_t booster_id,
  uint8_t command,
  const std::array<int32_t, 5> & payload,
  RequestContext context)
{
  if (!client->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(),
      *node_.get_clock(),
      5000,
      "%s service not available",
      service_name.c_str());
    return false;
  }

  auto request = std::make_shared<interfaces::srv::BoosterCmd::Request>();
  request->id = booster_id;
  request->cmd.value = command;
  request->payload = payload;

  const auto logger = node_.get_logger();
  auto future = client->async_send_request(request);
  const auto timeout =
    std::chrono::duration<double>(config_.command_ack_timeout_sec);
  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(
      logger,
      "%s timed out waiting for ACK for %s %s command for %s",
      service_name.c_str(),
      context.service_group.c_str(),
      context.device_name.c_str(),
      context.device_id.c_str());
    return false;
  }

  const auto response = future.get();
  if (!command_succeeded(response->result)) {
    RCLCPP_ERROR(
      logger,
      "%s rejected %s %s command for %s: HRESULT=%d ack=%d",
      service_name.c_str(),
      context.service_group.c_str(),
      context.device_name.c_str(),
      context.device_id.c_str(),
      response->result.value,
      response->ack_id);
    return false;
  }

  RCLCPP_DEBUG(
    logger,
    "%s ACK %d for %s %s command for %s",
    service_name.c_str(),
    response->ack_id,
    context.service_group.c_str(),
    context.device_name.c_str(),
    context.device_id.c_str());
  return true;
}

bool DeviceDispatcher::send_compressor_request(
  uint8_t command,
  const std::array<int32_t, 5> & payload,
  RequestContext context)
{
  const auto service_name = config_.services.control_compressor_service_name;
  if (!control_compressor_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(),
      *node_.get_clock(),
      5000,
      "%s service not available",
      service_name.c_str());
    return false;
  }

  auto request = std::make_shared<interfaces::srv::CompressorCmd::Request>();
  request->id = config_.command_ids.compressor;
  request->cmd.value = command;
  request->payload = payload;

  const auto logger = node_.get_logger();
  auto future = control_compressor_client_->async_send_request(request);
  const auto timeout =
    std::chrono::duration<double>(config_.command_ack_timeout_sec);
  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(
      logger,
      "%s timed out waiting for ACK for %s %s command for %s",
      service_name.c_str(),
      context.service_group.c_str(),
      context.device_name.c_str(),
      context.device_id.c_str());
    return false;
  }

  const auto response = future.get();
  if (!command_succeeded(response->result)) {
    RCLCPP_ERROR(
      logger,
      "%s rejected %s %s command for %s: HRESULT=%d ack=%d",
      service_name.c_str(),
      context.service_group.c_str(),
      context.device_name.c_str(),
      context.device_id.c_str(),
      response->result.value,
      response->ack_id);
    return false;
  }

  RCLCPP_DEBUG(
    logger,
    "%s ACK %d for %s %s command for %s",
    service_name.c_str(),
    response->ack_id,
    context.service_group.c_str(),
    context.device_name.c_str(),
    context.device_id.c_str());
  return true;
}

}  // namespace compressor

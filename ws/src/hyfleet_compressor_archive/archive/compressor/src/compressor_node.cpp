#include "compressor/compressor_node.hpp"

#include "internal/compressor_action.hpp"
#include "internal/compressor_config.hpp"
#include "internal/diagnostics_publisher.hpp"
#include "internal/device_dispatcher.hpp"
#include "internal/telemetry_subscriber.hpp"

#include <chrono>
#include <exception>
#include <string>

namespace compressor
{

using namespace std::chrono_literals;

namespace
{
CompressorControl safe_stop_all_boosters()
{
  return CompressorControl{
    CompressorCommand::SAFE_STOP,
    CompressorTarget::SYNC_BOOSTERS,
    0.0};
}

void apply_status_message(CompressorStatus & status, const std::string & message)
{
  status.low_booster.message = message;
  status.high_booster.message = message;
}
}  // namespace

CompressorNode::CompressorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("compressor_twin", options)
{
  RCLCPP_INFO(get_logger(), "CompressorNode constructed");
}

CompressorNode::~CompressorNode() = default;

CompressorNode::CallbackReturn CompressorNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring compressor...");

  try {
    const auto config = load_compressor_config(*this);
    action_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    telemetry_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    booster_manager_ = std::make_unique<BoosterManager>(
      config.low_booster,
      config.high_booster);
    device_dispatcher_ = std::make_unique<DeviceDispatcher>(
      *this,
      config.device_dispatcher,
      client_callback_group_);
    diagnostics_publisher_ =
      std::make_unique<DiagnosticsPublisher>(*this, config.ros.diagnostics_topic);
    diagnostics_publisher_->configure();
    safety_monitor_ = std::make_unique<SafetyMonitor>(config.safety);
    telemetry_timeout_sec_ = config.safety.telemetry_timeout_sec;
    compressor_action_ = std::make_unique<CompressorAction>(
      *this,
      config.ros.control_action_name,
      config.command_limits);
    compressor_action_->configure(action_callback_group_);
    telemetry_subscriber_ = std::make_unique<TelemetrySubscriber>(
      *this,
      config.ros.telemetry_topic,
      config.low_booster_telemetry,
      config.high_booster_telemetry,
      [this](const CompressorTelemetryUpdate & telemetry) {
        handle_telemetry(telemetry);
      });
    telemetry_subscriber_->configure(telemetry_callback_group_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to configure compressor: %s", error.what());
    reset_ros_entities();
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Compressor configured");
  return CallbackReturn::SUCCESS;
}

CompressorNode::CallbackReturn CompressorNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (diagnostics_publisher_) {
    diagnostics_publisher_->activate();
  }
  control_timer_ = create_wall_timer(
    1s,
    [this]() {
      process_pending_control();
    });
  if (compressor_action_) {
    compressor_action_->set_accepting_goals(true);
  }
  RCLCPP_INFO(get_logger(), "Compressor active");
  return CallbackReturn::SUCCESS;
}

CompressorNode::CallbackReturn CompressorNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating compressor...");
  control_timer_.reset();
  if (compressor_action_) {
    compressor_action_->set_accepting_goals(false);
    compressor_action_->abort_active_goals("Compressor lifecycle node deactivated");
  }
  dispatch_safe_stop("Compressor lifecycle node deactivated");
  if (diagnostics_publisher_) {
    diagnostics_publisher_->deactivate();
  }
  RCLCPP_INFO(get_logger(), "Compressor deactivated");
  return CallbackReturn::SUCCESS;
}

CompressorNode::CallbackReturn CompressorNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up compressor...");
  dispatch_safe_stop("Compressor lifecycle node cleanup");
  reset_ros_entities();
  return CallbackReturn::SUCCESS;
}

CompressorNode::CallbackReturn CompressorNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down compressor...");
  dispatch_safe_stop("Compressor lifecycle node shutdown");
  reset_ros_entities();
  return CallbackReturn::SUCCESS;
}

void CompressorNode::process_pending_control()
{
  if (!compressor_action_ || !booster_manager_ || !device_dispatcher_) {
    return;
  }

  const auto control = compressor_action_->take_pending_control();
  DeviceCommands commands;
  CompressorStatus status;
  SafetyStatus safety_status;
  bool telemetry_received = false;
  uint8_t telemetry_mode = 0;
  builtin_interfaces::msg::Time telemetry_timestamp;
  {
    std::scoped_lock lock(booster_manager_mutex_);
    safety_status = latest_safety_status_;
    if (safety_monitor_ && !telemetry_received_) {
      safety_status = safety_monitor_->telemetry_missing();
    } else if (telemetry_received_ &&
      (now() - latest_telemetry_receive_time_).seconds() > telemetry_timeout_sec_)
    {
      safety_status.inhibited = true;
      safety_status.message = "Compressor telemetry stale, inhibiting compressor operation";
      safety_status.commands = {};
    }

    if (control && !safety_status.inhibited) {
      booster_manager_->apply_control(*control);
    }
    if (safety_status.inhibited) {
      booster_manager_->apply_control(safe_stop_all_boosters());
    }

    commands = booster_manager_->update();
    append_device_commands(commands, safety_status.commands);
    status = booster_manager_->status();
    if (safety_status.inhibited) {
      apply_status_message(status, safety_status.message);
    }
    telemetry_received = telemetry_received_;
    telemetry_mode = latest_telemetry_mode_;
    telemetry_timestamp = latest_telemetry_timestamp_;
  }
  const bool dispatched = device_dispatcher_->dispatch(commands);
  if (!dispatched) {
    RCLCPP_ERROR(get_logger(), "Device command dispatch failed; requesting safe stop");
    dispatch_safe_stop("Device command dispatch failed");
  } else if (safety_status.inhibited) {
    compressor_action_->abort_active_goals(safety_status.message);
  } else {
    compressor_action_->process_status(status);
  }
  publish_diagnostics(status, telemetry_received, telemetry_mode, telemetry_timestamp);
}

void CompressorNode::handle_telemetry(const CompressorTelemetryUpdate & telemetry)
{
  CompressorStatus status;
  bool telemetry_received = false;
  uint8_t telemetry_mode = 0;
  builtin_interfaces::msg::Time telemetry_timestamp;
  {
    std::scoped_lock lock(booster_manager_mutex_);
    if (!booster_manager_) {
      return;
    }
    latest_telemetry_mode_ = telemetry.mode;
    latest_telemetry_timestamp_ = telemetry.timestamp;
    latest_telemetry_receive_time_ = now();
    telemetry_received_ = true;
    if (safety_monitor_) {
      latest_safety_status_ = safety_monitor_->evaluate(telemetry);
    }
    booster_manager_->set_telemetry(telemetry.low_booster, telemetry.high_booster);
    status = booster_manager_->status();
    if (latest_safety_status_.inhibited) {
      apply_status_message(status, latest_safety_status_.message);
    }
    telemetry_received = telemetry_received_;
    telemetry_mode = latest_telemetry_mode_;
    telemetry_timestamp = latest_telemetry_timestamp_;
  }
  publish_diagnostics(status, telemetry_received, telemetry_mode, telemetry_timestamp);
}

void CompressorNode::publish_diagnostics(
  const CompressorStatus & status,
  bool telemetry_received,
  uint8_t telemetry_mode,
  const builtin_interfaces::msg::Time & telemetry_timestamp) const
{
  if (diagnostics_publisher_) {
    diagnostics_publisher_->publish(
      status,
      telemetry_received,
      telemetry_mode,
      telemetry_timestamp);
  }
}

bool CompressorNode::dispatch_safe_stop(const std::string & reason)
{
  if (!booster_manager_ || !device_dispatcher_) {
    return true;
  }

  CompressorStatus status;
  bool telemetry_received = false;
  uint8_t telemetry_mode = 0;
  builtin_interfaces::msg::Time telemetry_timestamp;
  bool all_dispatched = true;

  for (int attempt = 0; attempt < 4; ++attempt) {
    DeviceCommands commands;
    {
      std::scoped_lock lock(booster_manager_mutex_);
      booster_manager_->apply_control(safe_stop_all_boosters());
      commands = booster_manager_->update();
      status = booster_manager_->status();
      telemetry_received = telemetry_received_;
      telemetry_mode = latest_telemetry_mode_;
      telemetry_timestamp = latest_telemetry_timestamp_;
    }

    all_dispatched = device_dispatcher_->dispatch(commands) && all_dispatched;
    if (!status.low_booster.active && !status.high_booster.active) {
      break;
    }
  }

  if (!all_dispatched) {
    RCLCPP_ERROR(get_logger(), "Safe-stop dispatch failed: %s", reason.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Safe-stop command dispatched: %s", reason.c_str());
  }
  publish_diagnostics(status, telemetry_received, telemetry_mode, telemetry_timestamp);
  return all_dispatched;
}

void CompressorNode::reset_ros_entities()
{
  control_timer_.reset();
  if (telemetry_subscriber_) {
    telemetry_subscriber_->reset();
  }
  if (compressor_action_) {
    compressor_action_->reset();
  }
  if (diagnostics_publisher_) {
    diagnostics_publisher_->reset();
  }
  telemetry_received_ = false;
  latest_telemetry_mode_ = 0;
  latest_telemetry_timestamp_ = builtin_interfaces::msg::Time();
  latest_telemetry_receive_time_ = rclcpp::Time();
  latest_safety_status_ = {};
  telemetry_subscriber_.reset();
  compressor_action_.reset();
  diagnostics_publisher_.reset();
  device_dispatcher_.reset();
  safety_monitor_.reset();
  booster_manager_.reset();
  client_callback_group_.reset();
  telemetry_callback_group_.reset();
  action_callback_group_.reset();
}

}  // namespace compressor

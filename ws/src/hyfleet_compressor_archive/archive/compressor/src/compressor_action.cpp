#include "internal/compressor_action.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace compressor
{

namespace
{
bool target_is_idle(
  const interfaces::action::ControlCompressor::Goal & goal,
  const CompressorStatus & status)
{
  if (goal.target == interfaces::action::ControlCompressor::Goal::HIGH_BOOSTER) {
    return !status.high_booster.active;
  }
  if (goal.target == interfaces::action::ControlCompressor::Goal::SYNC_BOOSTERS) {
    return !status.low_booster.active && !status.high_booster.active;
  }
  return !status.low_booster.active;
}

bool start_target_reached(
  const interfaces::action::ControlCompressor::Goal & goal,
  const CompressorStatus & status)
{
  if (goal.target == interfaces::action::ControlCompressor::Goal::HIGH_BOOSTER) {
    return status.high_booster.target_reached;
  }
  if (goal.target == interfaces::action::ControlCompressor::Goal::SYNC_BOOSTERS) {
    return status.low_booster.target_reached && status.high_booster.target_reached;
  }
  return status.low_booster.target_reached;
}

bool goal_is_complete(
  const interfaces::action::ControlCompressor::Goal & goal,
  const CompressorStatus & status)
{
  if (goal.command == interfaces::action::ControlCompressor::Goal::START) {
    return start_target_reached(goal, status);
  }
  return target_is_idle(goal, status);
}

bool start_goal_failed(
  const interfaces::action::ControlCompressor::Goal & goal,
  const CompressorStatus & status)
{
  return goal.command == interfaces::action::ControlCompressor::Goal::START &&
         target_is_idle(goal, status) &&
         !start_target_reached(goal, status);
}

std::pair<double, double> pressure_limits_for_target(
  uint8_t target,
  const CommandLimitsConfig & limits)
{
  if (target == interfaces::action::ControlCompressor::Goal::LOW_BOOSTER) {
    return {limits.low_booster_min_pressure_bar, limits.low_booster_max_pressure_bar};
  }
  if (target == interfaces::action::ControlCompressor::Goal::HIGH_BOOSTER) {
    return {limits.high_booster_min_pressure_bar, limits.high_booster_max_pressure_bar};
  }
  return {
    std::max(limits.low_booster_min_pressure_bar, limits.high_booster_min_pressure_bar),
    std::min(limits.low_booster_max_pressure_bar, limits.high_booster_max_pressure_bar)};
}
}  // namespace

CompressorAction::CompressorAction(
  rclcpp_lifecycle::LifecycleNode & node,
  std::string action_name,
  CommandLimitsConfig limits)
: node_(node),
  action_name_(std::move(action_name)),
  limits_(limits)
{
}

void CompressorAction::configure(rclcpp::CallbackGroup::SharedPtr callback_group)
{
  callback_group_ = callback_group;
  control_server_ = rclcpp_action::create_server<ControlCompressor>(
    node_.get_node_base_interface(),
    node_.get_node_clock_interface(),
    node_.get_node_logging_interface(),
    node_.get_node_waitables_interface(),
    action_name_,
    [this](const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ControlCompressor::Goal> goal) {
      return handle_goal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleControlCompressor> goal_handle) {
      return handle_cancel(goal_handle);
    },
    [this](const std::shared_ptr<GoalHandleControlCompressor> goal_handle) {
      handle_accepted(goal_handle);
    },
    rcl_action_server_get_default_options(),
    callback_group);
}

void CompressorAction::reset()
{
  set_accepting_goals(false);
  abort_active_goals("Compressor action server reset");
  control_server_.reset();
}

void CompressorAction::set_accepting_goals(bool accepting)
{
  std::scoped_lock lock(mutex_);
  accepting_goals_ = accepting;
  if (!accepting_goals_) {
    pending_controls_.clear();
  }
}

std::optional<CompressorControl> CompressorAction::take_pending_control()
{
  std::scoped_lock lock(mutex_);
  if (pending_controls_.empty()) {
    return std::nullopt;
  }
  auto control = pending_controls_.front();
  pending_controls_.pop_front();
  return control;
}

void CompressorAction::process_status(const CompressorStatus & status)
{
  std::scoped_lock lock(mutex_);
  auto active_goal = active_goals_.begin();
  while (active_goal != active_goals_.end()) {
    publish_feedback(active_goal->goal_handle, status);
    const auto goal = active_goal->goal_handle->get_goal();
    if (goal && start_goal_failed(*goal, status)) {
      abort_goal(active_goal->goal_handle, "Compressor start stopped before target was reached");
      active_goal = active_goals_.erase(active_goal);
    } else if (goal && goal_is_complete(*goal, status)) {
      succeed_goal(active_goal->goal_handle);
      active_goal = active_goals_.erase(active_goal);
    } else {
      ++active_goal;
    }
  }
}

rclcpp_action::GoalResponse CompressorAction::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const ControlCompressor::Goal> goal)
{
  if (!goal) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::scoped_lock lock(mutex_);
    if (!accepting_goals_) {
      RCLCPP_WARN(node_.get_logger(), "Rejecting compressor goal while node is inactive");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  switch (goal->command) {
    case ControlCompressor::Goal::START:
      {
        const auto [min_pressure, max_pressure] =
          pressure_limits_for_target(goal->target, limits_);
        if (!std::isfinite(goal->target_pressure) ||
          goal->target_pressure < min_pressure ||
          goal->target_pressure > max_pressure)
        {
          RCLCPP_WARN(
            node_.get_logger(),
            "Rejecting compressor start target pressure %.2f outside [%.2f, %.2f]",
            goal->target_pressure,
            min_pressure,
            max_pressure);
          return rclcpp_action::GoalResponse::REJECT;
        }
        break;
      }
    case ControlCompressor::Goal::STOP:
    case ControlCompressor::Goal::SAFE_STOP:
      break;
    default:
      return rclcpp_action::GoalResponse::REJECT;
  }

  switch (goal->target) {
    case ControlCompressor::Goal::LOW_BOOSTER:
    case ControlCompressor::Goal::HIGH_BOOSTER:
    case ControlCompressor::Goal::SYNC_BOOSTERS:
      break;
    default:
      return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CompressorAction::handle_cancel(
  const std::shared_ptr<GoalHandleControlCompressor> goal_handle)
{
  std::scoped_lock lock(mutex_);
  auto active_goal = active_goals_.begin();
  while (active_goal != active_goals_.end()) {
    if (active_goal->goal_handle == goal_handle) {
      remove_pending_controls_for_target(active_goal->target);
      const auto goal = active_goal->goal_handle->get_goal();
      if (goal) {
        pending_controls_.push_back(stop_control_for_goal(*goal));
      }
      cancel_goal(active_goal->goal_handle);
      active_goal = active_goals_.erase(active_goal);
    } else {
      ++active_goal;
    }
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void CompressorAction::handle_accepted(
  const std::shared_ptr<GoalHandleControlCompressor> goal_handle)
{
  std::scoped_lock lock(mutex_);
  const auto goal = goal_handle->get_goal();
  if (goal) {
    remove_pending_controls_for_target(goal->target);
    pending_controls_.push_back(control_from_goal(*goal));
    replace_overlapping_goals(goal_handle);
  }
  RCLCPP_INFO(node_.get_logger(), "Accepted compressor command");
}

void CompressorAction::abort_active_goals(const std::string & message)
{
  std::scoped_lock lock(mutex_);
  pending_controls_.clear();
  for (const auto & active_goal : active_goals_) {
    if (!active_goal.goal_handle) {
      continue;
    }
    auto result = std::make_shared<ControlCompressor::Result>();
    result->accepted = false;
    result->message = message;
    active_goal.goal_handle->abort(result);
  }
  active_goals_.clear();
}

void CompressorAction::cancel_goal(
  const std::shared_ptr<GoalHandleControlCompressor> & stored_goal)
{
  auto result = std::make_shared<ControlCompressor::Result>();
  result->accepted = false;
  result->message = "Compressor command cancelled";
  stored_goal->canceled(result);
}

void CompressorAction::remove_pending_controls_for_target(uint8_t target)
{
  auto pending_control = pending_controls_.begin();
  while (pending_control != pending_controls_.end()) {
    if (targets_overlap(static_cast<uint8_t>(pending_control->target), target)) {
      pending_control = pending_controls_.erase(pending_control);
    } else {
      ++pending_control;
    }
  }
}

void CompressorAction::replace_overlapping_goals(
  const std::shared_ptr<GoalHandleControlCompressor> & new_goal)
{
  const auto goal = new_goal->get_goal();
  if (!goal) {
    return;
  }

  auto active_goal = active_goals_.begin();
  while (active_goal != active_goals_.end()) {
    if (!targets_overlap(active_goal->target, goal->target)) {
      ++active_goal;
      continue;
    }
    auto result = std::make_shared<ControlCompressor::Result>();
    result->accepted = false;
    result->message = "Compressor command replaced";
    active_goal->goal_handle->abort(result);
    active_goal = active_goals_.erase(active_goal);
  }
  active_goals_.push_back({new_goal, goal->target});
}

void CompressorAction::publish_feedback(
  const std::shared_ptr<GoalHandleControlCompressor> & goal_handle,
  const CompressorStatus & status) const
{
  const auto goal = goal_handle->get_goal();
  double pressure = status.low_booster.pressure;

  if (goal && goal->target == ControlCompressor::Goal::HIGH_BOOSTER) {
    pressure = status.high_booster.pressure;
  } else if (goal && goal->target == ControlCompressor::Goal::SYNC_BOOSTERS) {
    pressure = std::min(status.low_booster.pressure, status.high_booster.pressure);
  }

  double percent_complete = 0.0;
  if (goal && goal->target_pressure > 0.0) {
    percent_complete =
      std::clamp((pressure / goal->target_pressure) * 100.0, 0.0, 100.0);
  }

  auto feedback = std::make_shared<ControlCompressor::Feedback>();
  feedback->pressure = pressure;
  feedback->percent_complete = percent_complete;

  goal_handle->publish_feedback(feedback);
}

void CompressorAction::abort_goal(
  const std::shared_ptr<GoalHandleControlCompressor> & stored_goal,
  const std::string & message) const
{
  auto result = std::make_shared<ControlCompressor::Result>();
  result->accepted = false;
  result->message = message;
  stored_goal->abort(result);
}

void CompressorAction::succeed_goal(
  const std::shared_ptr<GoalHandleControlCompressor> & stored_goal) const
{
  auto result = std::make_shared<ControlCompressor::Result>();
  result->accepted = true;
  result->message = "Compressor command target reached";
  stored_goal->succeed(result);
}

CompressorControl CompressorAction::control_from_goal(const ControlCompressor::Goal & goal)
{
  return CompressorControl{
    static_cast<CompressorCommand>(goal.command),
    static_cast<CompressorTarget>(goal.target),
    goal.target_pressure};
}

CompressorControl CompressorAction::stop_control_for_goal(const ControlCompressor::Goal & goal)
{
  return CompressorControl{
    CompressorCommand::STOP,
    static_cast<CompressorTarget>(goal.target),
    0.0};
}

bool CompressorAction::targets_overlap(uint8_t lhs, uint8_t rhs)
{
  if (lhs == ControlCompressor::Goal::SYNC_BOOSTERS ||
    rhs == ControlCompressor::Goal::SYNC_BOOSTERS)
  {
    return true;
  }
  return lhs == rhs;
}

}  // namespace compressor

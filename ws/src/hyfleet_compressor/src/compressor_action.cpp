#include "include/compressor_action.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace hyfleet_compressor
{

// ==============================================================================
// Constructor
// ==============================================================================

CompressorAction::CompressorAction(rclcpp_lifecycle::LifecycleNode & node, std::string action_name, CommandLimitsConfig limits):
  node_(node),
  action_name_(std::move(action_name)),
  limits_(limits) {}

// ==============================================================================
// lifecycle helers
// ==============================================================================

void CompressorAction::configure(rclcpp::CallbackGroup::SharedPtr callback_group) {

  if (!callback_group) {
      throw std::invalid_argument("CompressorAction requires a dedicated callback group");
  }
  callback_group_ = callback_group;

  // Spawn action server
  control_server_ = rclcpp_action::create_server<ControlCompressor>(
    node_.get_node_base_interface(),
    node_.get_node_clock_interface(),
    node_.get_node_logging_interface(),
    node_.get_node_waitables_interface(),
    action_name_,
    [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ControlCompressor::Goal> goal) {
      return handle_goal(uuid, goal); },
    [this](const std::shared_ptr<GoalHandleControlCompressor> goal_handle) {
      return handle_cancel(goal_handle);},
    [this](const std::shared_ptr<GoalHandleControlCompressor> goal_handle) {
      handle_accepted(goal_handle); },
    rcl_action_server_get_default_options(),
    callback_group);
}

void CompressorAction::unconfigure(){
    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto & active_goal : active_goals_) {
        if (!active_goal.goal_handle || active_goal.goal_handle != nullptr) { continue; }

        auto result = std::make_shared<ControlCompressor::Result>();
        result->accepted = false;
        result->message = "Compressor action unconfigured";

        active_goal.goal_handle->abort(result);
    }

    active_goals_.clear();
    control_server_.reset();
    callback_group_.reset();
}

void CompressorAction::toggle_enable(bool state) {
  accepting_goals_ = state;
}

// ==============================================================================
// Action server responses
// ==============================================================================
rclcpp_action::GoalResponse CompressorAction::handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ControlCompressor::Goal> goal){
  if (!goal) { return rclcpp_action::GoalResponse::REJECT; }

  {
    std::scoped_lock lock(mutex_);
    if (!accepting_goals_) {
      RCLCPP_WARN(node_.get_logger(), "Rejecting compressor goal while node is inactive");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CompressorAction::handle_cancel(const std::shared_ptr<GoalHandleControlCompressor> goal_handle) {
  std::scoped_lock lock(mutex_);
  auto active_goal = active_goals_.begin();
  while (active_goal != active_goals_.end()) {
    if (active_goal->goal_handle == goal_handle) {
    }
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void CompressorAction::handle_accepted(const std::shared_ptr<GoalHandleControlCompressor> goal_handle) {
    const auto goal = goal_handle->get_goal();
    if (!goal) {
        RCLCPP_WARN(node_.get_logger(), "Accepted compressor command has null goal");
        return;
    }

    {
        std::scoped_lock lock(mutex_);
        active_goals_.push_back(ActiveGoal{.goal_handle = goal_handle, .target = goal->target });
    }

    RCLCPP_INFO(node_.get_logger(), "Accepted compressor command: target=%u", goal->target);

}

// ==============================================================================
// Goal management
// ==============================================================================

void CompressorAction::cancel_goal(const std::shared_ptr<GoalHandleControlCompressor> & stored_goal) {
  auto result = std::make_shared<ControlCompressor::Result>();
  result->accepted = false;
  result->message = "Compressor command cancelled";
  stored_goal->canceled(result);
}

void CompressorAction::abort_goal(const std::shared_ptr<GoalHandleControlCompressor> & stored_goal, const std::string & message) const {
  auto result = std::make_shared<ControlCompressor::Result>();
  result->accepted = false;
  result->message = message;
  stored_goal->abort(result);
}

void CompressorAction::succeed_goal(const std::shared_ptr<GoalHandleControlCompressor> & stored_goal) const {
  auto result = std::make_shared<ControlCompressor::Result>();
  result->accepted = true;
  result->message = "Compressor command target reached";
  stored_goal->succeed(result);
}

void CompressorAction::publish_feedback(const std::shared_ptr<GoalHandleControlCompressor> & goal_handle,  const CompressorStatus & status) const {
  (void) status;
  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<ControlCompressor::Feedback>();
  feedback->pressure = 0.0;
  feedback->percent_complete = 0;

  goal_handle->publish_feedback(feedback);
}

// ==============================================================================
// Goal management
// ==============================================================================



}  // namespace hyfleet_compressor

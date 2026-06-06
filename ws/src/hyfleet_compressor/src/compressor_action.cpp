#include "include/compressor_action.hpp"

#include <stdexcept>
#include <utility>

namespace hyfleet_compressor
{

// ==============================================================================
// Constructor
// ==============================================================================

CompressorAction::CompressorAction(rclcpp_lifecycle::LifecycleNode & node, std::string action_name)
: node_(node),
  action_name_(std::move(action_name))
{}

// ==============================================================================
// Lifecycle helpers
// ==============================================================================

void CompressorAction::configure(rclcpp::CallbackGroup::SharedPtr callback_group) {
    if (!callback_group) {
        throw std::invalid_argument("CompressorAction requires a dedicated callback group");
    }
    callback_group_ = callback_group;

    action_server_ = rclcpp_action::create_server<ControlCompressor>(
        node_.get_node_base_interface(),
        node_.get_node_clock_interface(),
        node_.get_node_logging_interface(),
        node_.get_node_waitables_interface(),
        action_name_,
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ControlCompressor::Goal> goal) {
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

void CompressorAction::unconfigure() {
    std::scoped_lock lock(mutex_);

    accepting_goals_ = false;
    goal_callback_ = nullptr;
    action_server_.reset();
    callback_group_.reset();
}

void CompressorAction::toggle_enable(bool state) {
    std::scoped_lock lock(mutex_);
    accepting_goals_ = state;
}

// ==============================================================================
// Action server callbacks
// ==============================================================================

rclcpp_action::GoalResponse CompressorAction::handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ControlCompressor::Goal> goal) {
    if (!goal) { return rclcpp_action::GoalResponse::REJECT; }

    std::scoped_lock lock(mutex_);
    if (!accepting_goals_) {
        RCLCPP_WARN(node_.get_logger(), "Rejecting coordinator goal — node is not active");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CompressorAction::handle_cancel(const std::shared_ptr<GoalHandleControlCompressor> /*goal_handle*/) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CompressorAction::handle_accepted(const std::shared_ptr<GoalHandleControlCompressor> goal_handle) {
    if (!goal_handle) {
        RCLCPP_WARN(node_.get_logger(), "Accepted coordinator goal handle is null");
        return;
    }

    const auto goal = goal_handle->get_goal();
    if (!goal) {
        RCLCPP_WARN(node_.get_logger(), "Accepted coordinator goal has null goal");
        return;
    }

    std::function<void(std::shared_ptr<GoalHandleControlCompressor>)> callback;
    {
        std::scoped_lock lock(mutex_);
        callback = goal_callback_;
    }

    RCLCPP_INFO(node_.get_logger(), "Accepted coordinator command: cmd=%u target=%u pressure=%.1f", 
        static_cast<unsigned>(goal->command), static_cast<unsigned>(goal->target), goal->target_pressure);

    if (callback) { callback(goal_handle); }
}

// ==============================================================================
// Goal management
// ==============================================================================

void CompressorAction::cancel_goal(const std::shared_ptr<GoalHandleControlCompressor> & goal_handle) {
    auto result = std::make_shared<ControlCompressor::Result>();
    result->accepted = false;
    result->message = "coordinator command cancelled";
    goal_handle->canceled(result);
}

void CompressorAction::abort_goal(const std::shared_ptr<GoalHandleControlCompressor> & goal_handle, const std::string & message) const {
    auto result = std::make_shared<ControlCompressor::Result>();
    result->accepted = false;
    result->message = message;
    goal_handle->abort(result);
}

void CompressorAction::succeed_goal(const std::shared_ptr<GoalHandleControlCompressor> & goal_handle) const {
    auto result = std::make_shared<ControlCompressor::Result>();
    result->accepted = true;
    result->message = "coordinator command completed";
    goal_handle->succeed(result);
}

void CompressorAction::set_goal_callback(std::function<void(std::shared_ptr<GoalHandleControlCompressor>)> cb) {
    std::scoped_lock lock(mutex_);
    goal_callback_ = std::move(cb);
}

}  // namespace hyfleet_compressor

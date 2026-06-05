#include "include/booster_action.hpp"

#include <utility>

namespace hyfleet_booster
{

// ==============================================================================
// Constructor
// ==============================================================================

BoosterAction::BoosterAction(rclcpp_lifecycle::LifecycleNode & node, std::string action_name)
: node_(node),
  action_name_(std::move(action_name))
{}

// ==============================================================================
// Lifecycle helpers
// ==============================================================================

void BoosterAction::configure(rclcpp::CallbackGroup::SharedPtr callback_group) {
    if (!callback_group) {
        throw std::invalid_argument("BoosterAction requires a dedicated callback group");
    }
    callback_group_ = callback_group;

    action_server_ = rclcpp_action::create_server<ControlBooster>(
        node_.get_node_base_interface(),
        node_.get_node_clock_interface(),
        node_.get_node_logging_interface(),
        node_.get_node_waitables_interface(),
        action_name_,
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ControlBooster::Goal> goal) {
            return handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandleControlBooster> goal_handle) {
            return handle_cancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleControlBooster> goal_handle) {
            handle_accepted(goal_handle);
        },
        rcl_action_server_get_default_options(),
        callback_group);
}

void BoosterAction::unconfigure() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (active_goal_) {
        auto result = std::make_shared<ControlBooster::Result>();
        result->accepted = false;
        result->message = "BoosterAction unconfigured";
        active_goal_->abort(result);
        active_goal_.reset();
    }

    action_server_.reset();
    callback_group_.reset();
}

void BoosterAction::toggle_enable(bool state) {
    accepting_goals_ = state;
}

// ==============================================================================
// Action server callbacks
// ==============================================================================

rclcpp_action::GoalResponse BoosterAction::handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ControlBooster::Goal> goal) {
    if (!goal) { return rclcpp_action::GoalResponse::REJECT; }

    std::scoped_lock lock(mutex_);
    if (!accepting_goals_) {
        RCLCPP_WARN(node_.get_logger(), "Rejecting booster goal — node is not active");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BoosterAction::handle_cancel( const std::shared_ptr<GoalHandleControlBooster> /*goal_handle*/) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void BoosterAction::handle_accepted(const std::shared_ptr<GoalHandleControlBooster> goal_handle) {
    const auto goal = goal_handle->get_goal();
    if (!goal) {
        RCLCPP_WARN(node_.get_logger(), "Accepted booster goal has null goal");
        return;
    }

    {
        std::scoped_lock lock(mutex_);
        active_goal_ = goal_handle;
    }

    RCLCPP_INFO(node_.get_logger(), "Accepted booster command: cmd=%u pressure=%.1f",
        goal->command, goal->target_pressure);

    // TODO: write goal fields to blackboard and start BT tick timer
}

// ==============================================================================
// Goal management
// ==============================================================================

void BoosterAction::cancel_goal(const std::shared_ptr<GoalHandleControlBooster> & goal_handle) {
    auto result = std::make_shared<ControlBooster::Result>();
    result->accepted = false;
    result->message = "booster command cancelled";
    goal_handle->canceled(result);
}

void BoosterAction::abort_goal(const std::shared_ptr<GoalHandleControlBooster> & goal_handle, const std::string & message) const {
    auto result = std::make_shared<ControlBooster::Result>();
    result->accepted = false;
    result->message = message;
    goal_handle->abort(result);
}

void BoosterAction::succeed_goal(const std::shared_ptr<GoalHandleControlBooster> & goal_handle) const {
    auto result = std::make_shared<ControlBooster::Result>();
    result->accepted = true;
    result->message = "booster command completed";
    goal_handle->succeed(result);
}

void BoosterAction::publish_feedback(const std::shared_ptr<GoalHandleControlBooster> & goal_handle) const {
    auto feedback = std::make_shared<ControlBooster::Feedback>();
    feedback->pressure = 0.0;
    feedback->percent_complete = 0.0;
    goal_handle->publish_feedback(feedback);
}

}  // namespace hyfleet_booster

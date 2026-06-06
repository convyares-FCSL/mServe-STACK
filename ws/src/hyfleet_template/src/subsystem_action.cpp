#include "include/subsystem_action.hpp"

#include <stdexcept>
#include <utility>

// TODO: rename namespace to hyfleet_<name>
namespace hyfleet_subsystem
{

SubsystemAction::SubsystemAction(rclcpp_lifecycle::LifecycleNode & node, std::string action_name)
: node_(node),
  action_name_(std::move(action_name))
{}

void SubsystemAction::configure(rclcpp::CallbackGroup::SharedPtr callback_group) {
    if (!callback_group) {
        throw std::invalid_argument("SubsystemAction requires a dedicated callback group");
    }
    callback_group_ = callback_group;

    action_server_ = rclcpp_action::create_server<ControlSubsystem>(
        node_.get_node_base_interface(),
        node_.get_node_clock_interface(),
        node_.get_node_logging_interface(),
        node_.get_node_waitables_interface(),
        action_name_,
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ControlSubsystem::Goal> goal) {
            return handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandleControlSubsystem> goal_handle) {
            return handle_cancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleControlSubsystem> goal_handle) {
            handle_accepted(goal_handle);
        },
        rcl_action_server_get_default_options(),
        callback_group);
}

void SubsystemAction::unconfigure() {
    std::scoped_lock lock(mutex_);
    accepting_goals_ = false;
    goal_callback_ = nullptr;
    action_server_.reset();
    callback_group_.reset();
}

void SubsystemAction::toggle_enable(bool state) {
    std::scoped_lock lock(mutex_);
    accepting_goals_ = state;
}

rclcpp_action::GoalResponse SubsystemAction::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ControlSubsystem::Goal> goal)
{
    if (!goal) { return rclcpp_action::GoalResponse::REJECT; }
    std::scoped_lock lock(mutex_);
    if (!accepting_goals_) {
        // TODO: update log prefix with subsystem name
        RCLCPP_WARN(node_.get_logger(), "Rejecting goal — node is not active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SubsystemAction::handle_cancel(
    const std::shared_ptr<GoalHandleControlSubsystem> /*goal_handle*/)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SubsystemAction::handle_accepted(
    const std::shared_ptr<GoalHandleControlSubsystem> goal_handle)
{
    if (!goal_handle) {
        RCLCPP_WARN(node_.get_logger(), "Accepted goal handle is null");
        return;
    }
    const auto goal = goal_handle->get_goal();
    if (!goal) {
        RCLCPP_WARN(node_.get_logger(), "Accepted goal is null");
        return;
    }

    std::function<void(std::shared_ptr<GoalHandleControlSubsystem>)> callback;
    {
        std::scoped_lock lock(mutex_);
        callback = goal_callback_;
    }

    // TODO: update log message to show relevant goal fields
    RCLCPP_INFO(node_.get_logger(), "Accepted command: cmd=%u",
        static_cast<unsigned>(goal->command));

    if (callback) { callback(goal_handle); }
}

void SubsystemAction::cancel_goal(
    const std::shared_ptr<GoalHandleControlSubsystem> & goal_handle)
{
    auto result = std::make_shared<ControlSubsystem::Result>();
    result->accepted = false;
    result->message = "command cancelled";
    goal_handle->canceled(result);
}

void SubsystemAction::abort_goal(
    const std::shared_ptr<GoalHandleControlSubsystem> & goal_handle,
    const std::string & message) const
{
    auto result = std::make_shared<ControlSubsystem::Result>();
    result->accepted = false;
    result->message = message;
    goal_handle->abort(result);
}

void SubsystemAction::succeed_goal(
    const std::shared_ptr<GoalHandleControlSubsystem> & goal_handle) const
{
    auto result = std::make_shared<ControlSubsystem::Result>();
    result->accepted = true;
    result->message = "command completed";
    goal_handle->succeed(result);
}

void SubsystemAction::set_goal_callback(
    std::function<void(std::shared_ptr<GoalHandleControlSubsystem>)> cb)
{
    std::scoped_lock lock(mutex_);
    goal_callback_ = std::move(cb);
}

}  // namespace hyfleet_subsystem

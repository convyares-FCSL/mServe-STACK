#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <functional>

#include <rclcpp/callback_group.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// TODO: replace with your action header
#include "mserve_interfaces/action/control_subsystem.hpp"

// TODO: rename namespace to hyfleet_<name>
namespace hyfleet_subsystem
{
// TODO: replace ControlSubsystem with your action type
using ControlSubsystem = mserve_interfaces::action::ControlSubsystem;
using GoalHandleControlSubsystem = rclcpp_action::ServerGoalHandle<ControlSubsystem>;

// TODO: rename class to <Name>Action
class SubsystemAction {
public:
    explicit SubsystemAction(rclcpp_lifecycle::LifecycleNode & node, std::string action_name = "subsystem_action");

    // Lifecycle
    void configure(rclcpp::CallbackGroup::SharedPtr callback_group);
    void unconfigure();
    void toggle_enable(bool state);

    // Used by subsystem_node.cpp
    void set_goal_callback(std::function<void(std::shared_ptr<GoalHandleControlSubsystem>)> cb);
    void succeed_goal(const std::shared_ptr<GoalHandleControlSubsystem> & goal_handle) const;
    void abort_goal(const std::shared_ptr<GoalHandleControlSubsystem> & goal_handle, const std::string & message) const;

private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ControlSubsystem::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleControlSubsystem> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleControlSubsystem> goal_handle);
    void cancel_goal(const std::shared_ptr<GoalHandleControlSubsystem> & goal_handle);

    std::function<void(std::shared_ptr<GoalHandleControlSubsystem>)> goal_callback_;
    bool accepting_goals_ = false;

    rclcpp_lifecycle::LifecycleNode & node_;
    rclcpp_action::Server<ControlSubsystem>::SharedPtr action_server_;
    std::string action_name_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    mutable std::mutex mutex_;
};

}  // namespace hyfleet_subsystem

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/callback_group.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "mserve_interfaces/action/control_booster.hpp"

namespace hyfleet_booster
{

using ControlBooster = mserve_interfaces::action::ControlBooster;
using GoalHandleControlBooster = rclcpp_action::ServerGoalHandle<ControlBooster>;

class BoosterAction {
public:
    explicit BoosterAction(rclcpp_lifecycle::LifecycleNode & node, std::string action_name = "booster_action");

    void configure(rclcpp::CallbackGroup::SharedPtr callback_group);
    void unconfigure();
    void toggle_enable(bool state);

private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ControlBooster::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleControlBooster> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleControlBooster> goal_handle);

    void cancel_goal(const std::shared_ptr<GoalHandleControlBooster> & goal_handle);
    void abort_goal(const std::shared_ptr<GoalHandleControlBooster> & goal_handle, const std::string & message) const;
    void succeed_goal(const std::shared_ptr<GoalHandleControlBooster> & goal_handle) const;
    void publish_feedback(const std::shared_ptr<GoalHandleControlBooster> & goal_handle) const;

    rclcpp_lifecycle::LifecycleNode & node_;
    rclcpp_action::Server<ControlBooster>::SharedPtr action_server_;
    std::string action_name_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    mutable std::mutex mutex_;

    std::shared_ptr<GoalHandleControlBooster> active_goal_;
    bool accepting_goals_ = false;
};

}  // namespace hyfleet_booster

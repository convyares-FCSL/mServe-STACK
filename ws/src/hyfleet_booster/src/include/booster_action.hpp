#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <functional>

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

    // Exposed for lifecycle node
    void configure(rclcpp::CallbackGroup::SharedPtr callback_group);
    void unconfigure();
    void toggle_enable(bool state);
    
    // Exposed for BT.cpp
    void set_goal_callback(std::function<void(std::shared_ptr<GoalHandleControlBooster>)> cb);
    void succeed_goal(const std::shared_ptr<GoalHandleControlBooster> & goal_handle) const;
    void abort_goal(const std::shared_ptr<GoalHandleControlBooster> & goal_handle, const std::string & message) const;

private:
    // Goal Response
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ControlBooster::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleControlBooster> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleControlBooster> goal_handle);

    // Handle Goal
    void cancel_goal(const std::shared_ptr<GoalHandleControlBooster> & goal_handle);
    void publish_feedback(const std::shared_ptr<GoalHandleControlBooster> & goal_handle) const;
    std::function<void(std::shared_ptr<GoalHandleControlBooster>)> goal_callback_;
    bool accepting_goals_ = false;

    // Parent node
    rclcpp_lifecycle::LifecycleNode & node_;

    // Action server
    rclcpp_action::Server<ControlBooster>::SharedPtr action_server_;
    std::string action_name_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    mutable std::mutex mutex_;
      
};

}  // namespace hyfleet_booster

#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/callback_group.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "mserve_interfaces/action/control_compressor.hpp"
#include "include/compressor_config.hpp"
#include "include/compressor_types.hpp"

namespace hyfleet_compressor
{

using ControlCompressor = mserve_interfaces::action::ControlCompressor;
using GoalHandleControlCompressor = rclcpp_action::ServerGoalHandle<ControlCompressor>;

struct ActiveGoal
{
    std::shared_ptr<GoalHandleControlCompressor> goal_handle;
    uint8_t target = 0;
};
  
class CompressorAction {
public:
    explicit CompressorAction(rclcpp_lifecycle::LifecycleNode & node, std::string action_name = "control_compressor", CommandLimitsConfig limits = {});

    // Lifecycle helpers
    void configure(rclcpp::CallbackGroup::SharedPtr callback_group);
    void unconfigure();
    void toggle_enable(bool state);

private:
    // Action server responses
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ControlCompressor::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleControlCompressor> goal_handle); 
    void handle_accepted( const std::shared_ptr<GoalHandleControlCompressor> goal_handle);
        
    // Goal management
    void cancel_goal(const std::shared_ptr<GoalHandleControlCompressor> & stored_goal);
    void abort_goal(const std::shared_ptr<GoalHandleControlCompressor> & stored_goal, const std::string & message) const;
    void succeed_goal(const std::shared_ptr<GoalHandleControlCompressor> & stored_goal) const;
    void publish_feedback(const std::shared_ptr<GoalHandleControlCompressor> & goal_handle, const CompressorStatus & status) const;

    // Process Controls
    std::deque<CompressorControl> pending_controls_;
    std::optional<CompressorControl> take_pending_control();
    void process_status(const CompressorStatus & status);
    void abort_active_goals(const std::string & message);

    // Reference to parent node
    rclcpp_lifecycle::LifecycleNode & node_;
    
    // Action server
    rclcpp_action::Server<ControlCompressor>::SharedPtr control_server_;
    std::string action_name_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    CommandLimitsConfig limits_;
    mutable std::mutex mutex_;
    
    // Current goal handles
    std::vector<ActiveGoal> active_goals_;
    bool accepting_goals_ = false;

};

}  // namespace hyfleet_compressor

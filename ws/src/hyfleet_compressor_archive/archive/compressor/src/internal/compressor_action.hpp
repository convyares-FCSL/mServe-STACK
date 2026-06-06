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

#include "interfaces/action/control_compressor.hpp"

#include "internal/compressor_config.hpp"
#include "internal/compressor_types.hpp"

namespace compressor
{

class CompressorAction
{
public:
  using ControlCompressor = interfaces::action::ControlCompressor;
  using GoalHandleControlCompressor = rclcpp_action::ServerGoalHandle<ControlCompressor>;

  explicit CompressorAction(
    rclcpp_lifecycle::LifecycleNode & node,
    std::string action_name = "control_compressor",
    CommandLimitsConfig limits = {});

  void configure(rclcpp::CallbackGroup::SharedPtr callback_group = nullptr);
  void reset();
  void set_accepting_goals(bool accepting);
  void abort_active_goals(const std::string & message);
  std::optional<CompressorControl> take_pending_control();
  void process_status(const CompressorStatus & status);

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ControlCompressor::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleControlCompressor> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleControlCompressor> goal_handle);

  struct ActiveGoal
  {
    std::shared_ptr<GoalHandleControlCompressor> goal_handle;
    uint8_t target = 0;
  };

  void cancel_goal(const std::shared_ptr<GoalHandleControlCompressor> & stored_goal);
  void remove_pending_controls_for_target(uint8_t target);
  void replace_overlapping_goals(
    const std::shared_ptr<GoalHandleControlCompressor> & new_goal);
  void publish_feedback(
    const std::shared_ptr<GoalHandleControlCompressor> & goal_handle,
    const CompressorStatus & status) const;
  void abort_goal(
    const std::shared_ptr<GoalHandleControlCompressor> & stored_goal,
    const std::string & message) const;
  void succeed_goal(const std::shared_ptr<GoalHandleControlCompressor> & stored_goal) const;
  static CompressorControl control_from_goal(const ControlCompressor::Goal & goal);
  static CompressorControl stop_control_for_goal(const ControlCompressor::Goal & goal);
  static bool targets_overlap(uint8_t lhs, uint8_t rhs);

  rclcpp_lifecycle::LifecycleNode & node_;
  std::string action_name_;
  CommandLimitsConfig limits_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  mutable std::mutex mutex_;
  bool accepting_goals_ = false;
  std::vector<ActiveGoal> active_goals_;
  std::deque<CompressorControl> pending_controls_;
  rclcpp_action::Server<ControlCompressor>::SharedPtr control_server_;
};

}  // namespace compressor

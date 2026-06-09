#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <mserve_interfaces/msg/drive_status.hpp>
#include <mserve_interfaces/msg/wheel_feedback.hpp>
#include <mserve_interfaces/srv/drive_chain_cmd.hpp>

#include "mserve_drivechain/motor_feedback.hpp"

namespace mserve_drivechain {

// Forward-declare private implementation types (complete definitions in src/).
class DriveUart;
class CmdVelCache;

class DrivechainNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit DrivechainNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DrivechainNode() override;

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  // Params (drivechain_params.cpp)
  void declare_params();
  void load_params();
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> &);
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // BT
  void register_bt_nodes();
  void build_bt_trees();
  bool run_tree_sync(BT::Tree &, std::chrono::milliseconds timeout = std::chrono::milliseconds(3000));
  void tick_drive_tree();

  BT::BehaviorTreeFactory         factory_;
  std::shared_ptr<BT::Blackboard> blackboard_;
  std::array<BT::Tree, 4>         trees_;  // [0]=connect [1]=stop [2]=set_id [3]=drive

  // Service
  using DriveChainCmd = mserve_interfaces::srv::DriveChainCmd;
  void on_drivechain_cmd(DriveChainCmd::Request::SharedPtr, DriveChainCmd::Response::SharedPtr);
  rclcpp::Service<DriveChainCmd>::SharedPtr cmd_service_;
  rclcpp::CallbackGroup::SharedPtr          service_cbg_;

  // Hardware
  std::unique_ptr<DriveUart>   uart_;
  std::unique_ptr<CmdVelCache> cmd_vel_cache_;
  std::mutex                   uart_mutex_;

  // Publishers — lifecycle-managed, pointers placed on blackboard as std::function
  rclcpp_lifecycle::LifecyclePublisher<mserve_interfaces::msg::WheelFeedback>::SharedPtr wheel_feedback_pub_;
  rclcpp_lifecycle::LifecyclePublisher<mserve_interfaces::msg::DriveStatus>::SharedPtr   drive_status_pub_;

  // Drive tick
  rclcpp::TimerBase::SharedPtr drive_timer_;
  bool                         drive_active_ = false;
};

}  // namespace mserve_drivechain

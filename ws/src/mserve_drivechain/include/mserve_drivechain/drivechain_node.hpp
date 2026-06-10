#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <interfaces/msg/drive_motor_feedback.hpp>
#include <interfaces/msg/drive_status.hpp>
#include <interfaces/srv/drive.hpp>
#include <interfaces/srv/set_motor_id.hpp>

namespace mserve_drivechain {

// Forward-declare private implementation types (complete definitions in src/).
class DriveUart;
class DriveCommandStore;

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

  // Services — one per command concern
  void create_services();
  void on_connect(std_srvs::srv::Trigger::Request::SharedPtr,         std_srvs::srv::Trigger::Response::SharedPtr);
  void on_stop   (std_srvs::srv::Trigger::Request::SharedPtr,         std_srvs::srv::Trigger::Response::SharedPtr);
  void on_drive  (interfaces::srv::Drive::Request::SharedPtr,         interfaces::srv::Drive::Response::SharedPtr);
  void on_set_id (interfaces::srv::SetMotorId::Request::SharedPtr,    interfaces::srv::SetMotorId::Response::SharedPtr);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<interfaces::srv::Drive>::SharedPtr drive_service_;
  rclcpp::Service<interfaces::srv::SetMotorId>::SharedPtr  set_id_service_;
  rclcpp::CallbackGroup::SharedPtr service_cbg_;

  // Publishers — lifecycle-managed, pointers placed on blackboard as std::function
  void create_publishers();
  void create_motor_feedback(bool sim_mode);
  rclcpp_lifecycle::LifecyclePublisher<interfaces::msg::DriveMotorFeedback>::SharedPtr motor_feedback_pub_;
  rclcpp_lifecycle::LifecyclePublisher<interfaces::msg::DriveStatus>::SharedPtr        drive_status_pub_;

   // Hardware
  std::unique_ptr<DriveUart>         uart_;
  std::unique_ptr<DriveCommandStore> drive_cmd_store_;
  std::mutex                         uart_mutex_;
  std::string                        last_drive_log_;

  // Drive tick
  rclcpp::TimerBase::SharedPtr drive_timer_;
  bool                         drive_active_ = false;
};

}  // namespace mserve_drivechain

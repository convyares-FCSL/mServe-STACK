#pragma once

#include <memory>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <interfaces/msg/drive_motor_feedback.hpp>
#include <interfaces/msg/drive_status.hpp>
#include <interfaces/srv/drive.hpp>

namespace mserve_base {

// Forward-declare private implementation type (complete definition in src/).
class CmdVelStore;

class BaseNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit BaseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BaseNode() override;

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  // Params (base_params.cpp)
  void declare_params();
  void load_params();
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> &);
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // BT
  void register_bt_nodes();
  void build_bt_tree();
  void tick_drive_tree();
  void send_zero_drive();  // best-effort zero command on deactivate/shutdown

  BT::BehaviorTreeFactory         factory_;
  std::shared_ptr<BT::Blackboard> blackboard_;
  BT::Tree                        drive_tree_;

  // /cmd_vel in
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::unique_ptr<CmdVelStore> cmd_vel_store_;

  // mserve_drivechain motor_feedback in (odometry input)
  void on_motor_feedback(const interfaces::msg::DriveMotorFeedback::SharedPtr msg);
  rclcpp::Subscription<interfaces::msg::DriveMotorFeedback>::SharedPtr motor_feedback_sub_;

  // mserve_sensehat IMU in (odometry yaw input — see UpdateOdometry's comment
  // on odometry.use_imu_for_yaw for why gyro replaces wheel-derived yaw rate)
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Publishers — lifecycle-managed, exposed on blackboard as std::function
  void create_publishers();
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_safe_pub_;
  rclcpp_lifecycle::LifecyclePublisher<interfaces::msg::DriveStatus>::SharedPtr   base_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr        odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr   joint_state_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // mserve_drivechain ~/drive service client
  rclcpp::Client<interfaces::srv::Drive>::SharedPtr drive_client_;

  // Drive tick
  rclcpp::TimerBase::SharedPtr drive_timer_;
  bool                         drive_active_ = false;
};

}  // namespace mserve_base

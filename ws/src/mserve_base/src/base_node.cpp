#include "mserve_base/base_node.hpp"
#include "include/base_types.hpp"
#include "include/base_bt_nodes.hpp"
#include "mserve_utils/qos.hpp"
#include "mserve_utils/topics.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace mserve_base {

using DriveStatus         = interfaces::msg::DriveStatus;
using DriveService        = interfaces::srv::Drive;
using DriveMotorFeedback  = interfaces::msg::DriveMotorFeedback;

// ==============================================================================
// Construction / destruction
// ==============================================================================

BaseNode::BaseNode(const rclcpp::NodeOptions & options) : rclcpp_lifecycle::LifecycleNode("mserve_base", options) {
  declare_params();
  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) { return on_parameters(p); });
}

// Destructor defined in .cpp so unique_ptr<CmdVelStore> is destroyed when that type is complete.
BaseNode::~BaseNode() = default;

// ==============================================================================
// Lifecycle
// ==============================================================================

BaseNode::CallbackReturn BaseNode::on_configure(const rclcpp_lifecycle::State &) {
  try {
    blackboard_ = BT::Blackboard::create();
    load_params();

    // Runtime state keys
    blackboard_->set("drivechain_reachable", false);
    blackboard_->set("ros_logger", get_logger());

    cmd_vel_store_ = std::make_unique<CmdVelStore>();
    blackboard_->set("cmd_vel_store", cmd_vel_store_.get());

    const auto cmd_vel_topic      = mserve_topics::cmd_vel(*this);
    const auto cmd_vel_safe_topic = mserve_topics::cmd_vel_safe(*this);
    const auto cmd_qos             = mserve_qos::commands(*this);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic, cmd_qos,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { on_cmd_vel(msg); });

    // QoS matches mserve_drivechain's motor_feedback publisher (rclcpp::QoS(10)).
    motor_feedback_sub_ = create_subscription<DriveMotorFeedback>(
      "/mserve_drivechain/motor_feedback", rclcpp::QoS(10),
      [this](const DriveMotorFeedback::SharedPtr msg) { on_motor_feedback(msg); });

    // QoS matches mserve_sensehat's imu publisher (rclcpp::SensorDataQoS()).
    // topic_names.imu is pre-declared in declare_params() — see that
    // function's comment for why this can't declare it itself here.
    const auto imu_topic = get_parameter("topic_names.imu").as_string();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(msg); });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    create_publishers();

    drive_client_ = create_client<DriveService>("/mserve_drivechain/drive");
    blackboard_->set("drive_client", drive_client_);

    // Odometry integration state — reset on every (re)configure.
    blackboard_->set("odom_initialized", false);
    blackboard_->set("odom_x",     0.0);
    blackboard_->set("odom_y",     0.0);
    blackboard_->set("odom_theta", 0.0);

    register_bt_nodes();
    build_bt_tree();

    RCLCPP_INFO(get_logger(), "Configuring mserve_base: %s -> %s -> /mserve_drivechain/drive",
      cmd_vel_topic.c_str(), cmd_vel_safe_topic.c_str());

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "on_configure failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

BaseNode::CallbackReturn BaseNode::on_activate(const rclcpp_lifecycle::State &) {
  cmd_vel_safe_pub_->on_activate();
  base_status_pub_->on_activate();
  odom_pub_->on_activate();
  joint_state_pub_->on_activate();
  drive_active_ = true;

  double rate = 10.0;
  (void)blackboard_->get("feedback_rate", rate);
  drive_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate),
    [this]() { tick_drive_tree(); });

  RCLCPP_INFO(get_logger(), "mserve_base activated");
  return CallbackReturn::SUCCESS;
}

BaseNode::CallbackReturn BaseNode::on_deactivate(const rclcpp_lifecycle::State &) {
  drive_active_ = false;
  if (drive_timer_) { drive_timer_->cancel(); drive_timer_.reset(); }

  send_zero_drive();

  cmd_vel_safe_pub_->on_deactivate();
  base_status_pub_->on_deactivate();
  odom_pub_->on_deactivate();
  joint_state_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "mserve_base deactivated");
  return CallbackReturn::SUCCESS;
}

BaseNode::CallbackReturn BaseNode::on_cleanup(const rclcpp_lifecycle::State &) {
  if (drive_timer_) { drive_timer_->cancel(); drive_timer_.reset(); }
  drive_tree_ = {};
  drive_client_.reset();
  cmd_vel_sub_.reset();
  motor_feedback_sub_.reset();
  imu_sub_.reset();
  tf_broadcaster_.reset();
  cmd_vel_safe_pub_.reset();
  base_status_pub_.reset();
  odom_pub_.reset();
  joint_state_pub_.reset();
  cmd_vel_store_.reset();
  blackboard_.reset();
  RCLCPP_INFO(get_logger(), "mserve_base cleaned up");
  return CallbackReturn::SUCCESS;
}

BaseNode::CallbackReturn BaseNode::on_shutdown(const rclcpp_lifecycle::State &) {
  if (drive_timer_) { drive_timer_->cancel(); drive_timer_.reset(); }
  send_zero_drive();
  RCLCPP_INFO(get_logger(), "mserve_base shut down");
  return CallbackReturn::SUCCESS;
}

// ==============================================================================
// BT
// ==============================================================================

void BaseNode::register_bt_nodes() {
  factory_.registerNodeType<ApplyCmdVelSafety>("ApplyCmdVelSafety");
  factory_.registerNodeType<ComputeKinematics>("ComputeKinematics");
  factory_.registerNodeType<CallDriveService>("CallDriveService");
  factory_.registerNodeType<PublishBaseStatus>("PublishBaseStatus");
  factory_.registerNodeType<UpdateOdometry>("UpdateOdometry");
  factory_.registerNodeType<PublishOdometry>("PublishOdometry");
}

void BaseNode::build_bt_tree() {
  const std::string base =
    ament_index_cpp::get_package_share_directory("mserve_base") + "/trees/";
  drive_tree_ = factory_.createTreeFromFile(base + "drive_tree.xml", blackboard_);
}

void BaseNode::tick_drive_tree() {
  if (!drive_active_) return;
  drive_tree_.tickOnce();
}

// ==============================================================================
// cmd_vel / publishers / drivechain client
// ==============================================================================

void BaseNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_vel_store_->update(*msg);
}

void BaseNode::on_motor_feedback(const DriveMotorFeedback::SharedPtr msg) {
  blackboard_->set("motor_feedback", *msg);
}

void BaseNode::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  blackboard_->set("imu", *msg);
}

void BaseNode::create_publishers() {
  const auto cmd_vel_safe_topic = mserve_topics::cmd_vel_safe(*this);
  const auto cmd_qos             = mserve_qos::commands(*this);

  cmd_vel_safe_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_safe_topic, cmd_qos);
  base_status_pub_  = create_publisher<DriveStatus>(std::string(get_name()) + "/base_status", rclcpp::QoS(10));
  // Plain top-level topics, not namespaced under the node — Nav2/robot_localization
  // and RViz's defaults both expect /odom and /joint_states unprefixed.
  odom_pub_         = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));
  joint_state_pub_  = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10));

  blackboard_->set<std::function<void(const geometry_msgs::msg::Twist &)>>("publish_cmd_vel_safe",
    [this](const geometry_msgs::msg::Twist & msg) {
      if (cmd_vel_safe_pub_->is_activated()) cmd_vel_safe_pub_->publish(msg);
    });
  blackboard_->set<std::function<void(const DriveStatus &)>>("publish_base_status",
    [this](const DriveStatus & msg) {
      if (base_status_pub_->is_activated()) base_status_pub_->publish(msg);
    });
  blackboard_->set<std::function<void(const nav_msgs::msg::Odometry &)>>("publish_odom",
    [this](const nav_msgs::msg::Odometry & msg) {
      if (odom_pub_->is_activated()) odom_pub_->publish(msg);
    });
  blackboard_->set<std::function<void(const sensor_msgs::msg::JointState &)>>("publish_joint_states",
    [this](const sensor_msgs::msg::JointState & msg) {
      if (joint_state_pub_->is_activated()) joint_state_pub_->publish(msg);
    });
  blackboard_->set<std::function<void(const geometry_msgs::msg::TransformStamped &)>>("publish_odom_tf",
    [this](const geometry_msgs::msg::TransformStamped & msg) {
      tf_broadcaster_->sendTransform(msg);
    });
}

// Best-effort zero command sent on deactivate/shutdown so the robot doesn't
// keep moving if the BT tick loop stops.
void BaseNode::send_zero_drive() {
  if (cmd_vel_safe_pub_ && cmd_vel_safe_pub_->is_activated()) {
    cmd_vel_safe_pub_->publish(geometry_msgs::msg::Twist{});
  }

  if (!drive_client_ || !blackboard_ || !drive_client_->service_is_ready()) return;

  int left_id = 0, right_id = 0;
  (void)blackboard_->get("left_motor_id",  left_id);
  (void)blackboard_->get("right_motor_id", right_id);

  auto request = std::make_shared<DriveService::Request>();
  request->motor_commands.resize(2);
  request->motor_commands[0].motor_id = static_cast<uint8_t>(left_id);
  request->motor_commands[0].rpm      = 0;
  request->motor_commands[1].motor_id = static_cast<uint8_t>(right_id);
  request->motor_commands[1].rpm      = 0;
  drive_client_->async_send_request(request);
}

}  // namespace mserve_base

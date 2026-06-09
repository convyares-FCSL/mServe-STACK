#include "mserve_drivechain/drivechain_node.hpp"
#include "include/drivechain_uart.hpp"
#include "include/drivechain_bt_nodes.hpp"
#include "include/drivechain_cmd_vel_cache.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <thread>

namespace mserve_drivechain {

using namespace std::chrono_literals;
using WheelFeedback = mserve_interfaces::msg::WheelFeedback;
using DriveStatus   = mserve_interfaces::msg::DriveStatus;

// ==============================================================================
// Construction / destruction
// ==============================================================================

DrivechainNode::DrivechainNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("mserve_drivechain", options)
{
  declare_params();
  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) { return on_parameters(p); });
}

// Destructor must be defined in .cpp so unique_ptr<CmdVelCache> + unique_ptr<DriveUart>
// are destroyed when those types are complete.
DrivechainNode::~DrivechainNode() = default;

// ==============================================================================
// Lifecycle
// ==============================================================================

DrivechainNode::CallbackReturn DrivechainNode::on_configure(const rclcpp_lifecycle::State &) {
  try {
    // --- blackboard first: load_params writes to it ---
    blackboard_ = BT::Blackboard::create();
    load_params();

    // Initialise runtime keys
    blackboard_->set("uart_connected",  false);
    blackboard_->set("left_rpm",        0);
    blackboard_->set("right_rpm",       0);
    blackboard_->set("left_speed_fb",   0.0);
    blackboard_->set("right_speed_fb",  0.0);
    blackboard_->set("left_pos_fb",     0.0);
    blackboard_->set("right_pos_fb",    0.0);
    blackboard_->set("left_fault",      0);
    blackboard_->set("right_fault",     0);

    // --- hardware objects (constructed from blackboard values) ---
    bool sim_mode = true;
    (void)blackboard_->get("sim_mode", sim_mode);
    uart_ = std::make_unique<DriveUart>(sim_mode);

    cmd_vel_cache_ = std::make_unique<CmdVelCache>(*this, "cmd_vel");

    // Put pointers on blackboard for BT nodes
    blackboard_->set("uart",          uart_.get());
    blackboard_->set("cmd_vel_cache", cmd_vel_cache_.get());
    blackboard_->set("ros_logger",    get_logger());

    // --- lifecycle publishers (activate/deactivate with node state) ---
    wheel_feedback_pub_ = create_publisher<WheelFeedback>(
      std::string(get_name()) + "/wheel_feedback", rclcpp::QoS(10));
    drive_status_pub_ = create_publisher<DriveStatus>(
      std::string(get_name()) + "/drive_status", rclcpp::QoS(10));

    // Expose publish functions on the blackboard so BT nodes can call them.
    // The lambdas check is_activated() so nodes don't need to care about lifecycle.
    blackboard_->set<std::function<void(const WheelFeedback &)>>(
      "publish_wheel_feedback",
      [this](const WheelFeedback & msg) {
        if (wheel_feedback_pub_->is_activated()) wheel_feedback_pub_->publish(msg);
      });
    blackboard_->set<std::function<void(const DriveStatus &)>>(
      "publish_drive_status",
      [this](const DriveStatus & msg) {
        if (drive_status_pub_->is_activated()) drive_status_pub_->publish(msg);
      });

    // --- BT ---
    register_bt_nodes();
    build_bt_trees();

    // --- service (separate callback group so it can block during tree sync) ---
    service_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cmd_service_ = create_service<DriveChainCmd>(
      std::string(get_name()) + "/drivechain_cmd",
      [this](DriveChainCmd::Request::SharedPtr req, DriveChainCmd::Response::SharedPtr res) {
        on_drivechain_cmd(req, res);
      },
      rclcpp::ServicesQoS(), service_cbg_);

    bool sim = true;
    (void)blackboard_->get("sim_mode", sim);
    int left_id = 1, right_id = 2;
    (void)blackboard_->get("left_motor_id", left_id);
    (void)blackboard_->get("right_motor_id", right_id);
    RCLCPP_INFO(get_logger(), "Configured — backend: %s, motors L=%d R=%d",
      sim ? "sim" : "hardware", left_id, right_id);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "on_configure failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

DrivechainNode::CallbackReturn DrivechainNode::on_activate(const rclcpp_lifecycle::State &) {
  wheel_feedback_pub_->on_activate();
  drive_status_pub_->on_activate();
  drive_active_ = true;

  double rate = 10.0;
  (void)blackboard_->get("feedback_rate", rate);
  drive_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate),
    [this]() { tick_drive_tree(); });

  RCLCPP_INFO(get_logger(), "Activated — send CONNECT to ~/drivechain_cmd to open hardware");
  return CallbackReturn::SUCCESS;
}

DrivechainNode::CallbackReturn DrivechainNode::on_deactivate(const rclcpp_lifecycle::State &) {
  drive_active_ = false;
  if (drive_timer_) { drive_timer_->cancel(); drive_timer_.reset(); }

  {
    std::lock_guard<std::mutex> lock(uart_mutex_);
    if (uart_ && uart_->is_open()) run_tree_sync(trees_[1], 2000ms);
  }

  wheel_feedback_pub_->on_deactivate();
  drive_status_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

DrivechainNode::CallbackReturn DrivechainNode::on_cleanup(const rclcpp_lifecycle::State &) {
  if (drive_timer_) { drive_timer_->cancel(); drive_timer_.reset(); }
  trees_          = {};
  blackboard_.reset();
  cmd_vel_cache_.reset();
  uart_.reset();
  wheel_feedback_pub_.reset();
  drive_status_pub_.reset();
  cmd_service_.reset();
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

DrivechainNode::CallbackReturn DrivechainNode::on_shutdown(const rclcpp_lifecycle::State &) {
  if (drive_timer_) { drive_timer_->cancel(); drive_timer_.reset(); }
  if (uart_ && uart_->is_open()) {
    std::lock_guard<std::mutex> lock(uart_mutex_);
    run_tree_sync(trees_[1], 1000ms);
  }
  RCLCPP_INFO(get_logger(), "Shutdown");
  return CallbackReturn::SUCCESS;
}

// ==============================================================================
// BT
// ==============================================================================

void DrivechainNode::register_bt_nodes() {
  // Connect / stop tree nodes
  factory_.registerNodeType<UartOpen>("UartOpen");
  factory_.registerNodeType<OpenUart>("OpenUart");
  factory_.registerNodeType<CloseUart>("CloseUart");
  factory_.registerNodeType<PingMotor>("PingMotor");
  factory_.registerNodeType<SetMotorMode>("SetMotorMode");
  factory_.registerNodeType<StopMotor>("StopMotor");
  factory_.registerNodeType<SetMotorId>("SetMotorId");
  factory_.registerNodeType<MotorHealthy>("MotorHealthy");

  // Drive tree nodes
  factory_.registerNodeType<ComputeRpm>("ComputeRpm");
  factory_.registerNodeType<SetMotorSpeed>("SetMotorSpeed");
  factory_.registerNodeType<PublishWheelFeedback>("PublishWheelFeedback");
  factory_.registerNodeType<PublishDriveStatus>("PublishDriveStatus");
}

void DrivechainNode::build_bt_trees() {
  const std::string base =
    ament_index_cpp::get_package_share_directory("mserve_drivechain") + "/trees/";
  trees_[0] = factory_.createTreeFromFile(base + "connect_tree.xml", blackboard_);
  trees_[1] = factory_.createTreeFromFile(base + "stop_tree.xml",    blackboard_);
  trees_[2] = factory_.createTreeFromFile(base + "set_id_tree.xml",  blackboard_);
  trees_[3] = factory_.createTreeFromFile(base + "drive_tree.xml",   blackboard_);
}

bool DrivechainNode::run_tree_sync(BT::Tree & tree, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (status == BT::NodeStatus::RUNNING) {
    status = tree.tickOnce();
    if (status != BT::NodeStatus::RUNNING) break;
    if (std::chrono::steady_clock::now() >= deadline) { tree.haltTree(); return false; }
    std::this_thread::sleep_for(10ms);
  }
  tree.haltTree();
  return status == BT::NodeStatus::SUCCESS;
}

void DrivechainNode::tick_drive_tree() {
  if (!drive_active_) return;
  std::lock_guard<std::mutex> lock(uart_mutex_);
  trees_[3].tickOnce();
}

// ==============================================================================
// Service
// ==============================================================================

void DrivechainNode::on_drivechain_cmd( DriveChainCmd::Request::SharedPtr req, DriveChainCmd::Response::SharedPtr res) {
  if (!drive_active_) {
    res->success = false; res->message = "node not activated"; return;
  }

  std::lock_guard<std::mutex> lock(uart_mutex_);

  switch (req->command) {
    case DriveChainCmd::Request::CONNECT: {
      const bool ok = run_tree_sync(trees_[0], 5000ms);
      res->success = ok;
      bool sim = true; (void)blackboard_->get("sim_mode", sim);
      res->message = ok
        ? (sim ? "connected (sim)" : "connected (hardware)")
        : "connect_tree failed — check motor IDs and uart_device param";
      RCLCPP_INFO(get_logger(), "CONNECT %s", ok ? "OK" : "FAILED");
      break;
    }
    case DriveChainCmd::Request::STOP: {
      const bool ok = run_tree_sync(trees_[1], 2000ms);
      res->success = ok;
      res->message = ok ? "stopped" : "stop_tree failed";
      RCLCPP_INFO(get_logger(), "STOP %s", ok ? "OK" : "FAILED");
      break;
    }
    case DriveChainCmd::Request::SET_ID: {
      blackboard_->set("target_motor_id", static_cast<int>(req->motor_id));
      blackboard_->set("new_motor_id",    static_cast<int>(req->new_id));
      const bool ok = run_tree_sync(trees_[2], 5000ms);
      res->success = ok;
      res->message = ok
        ? "motor ID changed — disconnect and reconnect to confirm"
        : "set_id_tree failed — ensure only one motor is on the bus";
      RCLCPP_INFO(get_logger(), "SET_ID %d→%d %s", req->motor_id, req->new_id, ok ? "OK" : "FAILED");
      break;
    }
    default:
      res->success = false; res->message = "unknown command";
  }
}

}  // namespace mserve_drivechain

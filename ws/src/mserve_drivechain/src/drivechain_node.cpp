#include "mserve_drivechain/drivechain_node.hpp"
#include "include/drivechain_types.hpp"
#include "include/drivechain_uart.hpp"
#include "include/drivechain_bt_nodes.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <thread>

namespace mserve_drivechain {

using namespace std::chrono_literals;
using DriveMotorFeedback = interfaces::msg::DriveMotorFeedback;
using DriveStatus        = interfaces::msg::DriveStatus;
using Trigger            = std_srvs::srv::Trigger;
using DriveService       = interfaces::srv::Drive;
using SetIdService       = interfaces::srv::SetMotorId;

// ==============================================================================
// Construction / destruction
// ==============================================================================

DrivechainNode::DrivechainNode(const rclcpp::NodeOptions & options) : rclcpp_lifecycle::LifecycleNode("mserve_drivechain", options) {
  declare_params();
  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) { return on_parameters(p); });
}

// Destructor defined in .cpp so unique_ptr<DriveCommandStore> + unique_ptr<DriveUart> are destroyed when those types are complete.
DrivechainNode::~DrivechainNode() = default;

// ==============================================================================
// Lifecycle
// ==============================================================================

DrivechainNode::CallbackReturn DrivechainNode::on_configure(const rclcpp_lifecycle::State &) {
  try {
    blackboard_ = BT::Blackboard::create();
    load_params();

    // Runtime state keys
    blackboard_->set("uart_connected", false);
    blackboard_->set("motor_states",   std::vector<interfaces::msg::MotorState>{});

    // Hardware objects
    bool sim_mode = true;
    (void)blackboard_->get("sim_mode", sim_mode);
    uart_           = std::make_unique<DriveUart>(sim_mode);
    drive_cmd_store_ = std::make_unique<DriveCommandStore>();

    // Pointers on blackboard for BT nodes
    blackboard_->set("uart",            uart_.get());
    blackboard_->set("drive_cmd_store", drive_cmd_store_.get());
    blackboard_->set("ros_logger",      get_logger());

    // Create publishers
    create_publishers();

    // Build bt
    register_bt_nodes();
    build_bt_trees();

    // Create services and feedback
    create_services();
    create_motor_feedback(sim_mode);


  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "on_configure failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

DrivechainNode::CallbackReturn DrivechainNode::on_activate(const rclcpp_lifecycle::State &) {
  motor_feedback_pub_->on_activate();
  drive_status_pub_->on_activate();
  drive_active_ = true;

  double rate = 10.0;
  (void)blackboard_->get("feedback_rate", rate);
  drive_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate),
    [this]() { tick_drive_tree(); });

  RCLCPP_INFO(get_logger(), "Activated — call ~/connect to open hardware");
  return CallbackReturn::SUCCESS;
}

DrivechainNode::CallbackReturn DrivechainNode::on_deactivate(const rclcpp_lifecycle::State &) {
  drive_active_ = false;
  if (drive_timer_) { drive_timer_->cancel(); drive_timer_.reset(); }

  {
    std::lock_guard<std::mutex> lock(uart_mutex_);
    if (uart_ && uart_->is_open()) run_tree_sync(trees_[1], 2000ms);
  }

  motor_feedback_pub_->on_deactivate();
  drive_status_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

DrivechainNode::CallbackReturn DrivechainNode::on_cleanup(const rclcpp_lifecycle::State &) {
  if (drive_timer_) { drive_timer_->cancel(); drive_timer_.reset(); }
  trees_           = {};
  blackboard_.reset();
  drive_cmd_store_.reset();
  uart_.reset();
  motor_feedback_pub_.reset();
  drive_status_pub_.reset();
  connect_service_.reset();
  stop_service_.reset();
  drive_service_.reset();
  set_id_service_.reset();
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
  factory_.registerNodeType<UartOpen>("UartOpen");
  factory_.registerNodeType<OpenUart>("OpenUart");
  factory_.registerNodeType<CloseUart>("CloseUart");
  factory_.registerNodeType<PingMotor>("PingMotor");
  factory_.registerNodeType<SetMotorMode>("SetMotorMode");
  factory_.registerNodeType<StopMotor>("StopMotor");
  factory_.registerNodeType<SetMotorId>("SetMotorId");
  factory_.registerNodeType<MotorHealthy>("MotorHealthy");

  factory_.registerNodeType<ConnectAllMotors>("ConnectAllMotors");
  factory_.registerNodeType<StopAllMotors>("StopAllMotors");
  factory_.registerNodeType<SetAllMotors>("SetAllMotors");
  factory_.registerNodeType<PublishMotorFeedback>("PublishMotorFeedback");
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
// Service handlers
// ==============================================================================

void DrivechainNode::create_publishers(){
    // Lifecycle publishers; exposed on blackboard as std::function so BT nodes
  motor_feedback_pub_ = create_publisher<DriveMotorFeedback>(std::string(get_name()) + "/motor_feedback", rclcpp::QoS(10));
  drive_status_pub_ = create_publisher<DriveStatus>(std::string(get_name()) + "/drive_status", rclcpp::QoS(10));

  blackboard_->set<std::function<void(const DriveMotorFeedback &)>>( "publish_motor_feedback",
    [this](const DriveMotorFeedback & msg) {
      if (motor_feedback_pub_->is_activated()) motor_feedback_pub_->publish(msg); 
    });
  blackboard_->set<std::function<void(const DriveStatus &)>>( "publish_drive_status",
    [this](const DriveStatus & msg) {
      if (drive_status_pub_->is_activated()) drive_status_pub_->publish(msg);
    });
}

void DrivechainNode::create_services(){
    service_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    connect_service_ = create_service<Trigger>(
      std::string(get_name()) + "/connect",
      [this](Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res) {
        on_connect(req, res);
      },
      rclcpp::ServicesQoS(), service_cbg_);

    stop_service_ = create_service<Trigger>(
      std::string(get_name()) + "/stop",
      [this](Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res) {
        on_stop(req, res);
      },
      rclcpp::ServicesQoS(), service_cbg_);

    drive_service_ = create_service<DriveService>(
      std::string(get_name()) + "/drive",
      [this](DriveService::Request::SharedPtr req, DriveService::Response::SharedPtr res) {
        on_drive(req, res);
      },
      rclcpp::ServicesQoS(), service_cbg_);

    set_id_service_ = create_service<SetIdService>(
      std::string(get_name()) + "/set_motor_id",
      [this](SetIdService::Request::SharedPtr req, SetIdService::Response::SharedPtr res) {
        on_set_id(req, res);
      },
      rclcpp::ServicesQoS(), service_cbg_);
}

void DrivechainNode::on_connect( Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
  if (!drive_active_) { res->success = false; res->message = "node not activated"; return; }
  std::lock_guard<std::mutex> lock(uart_mutex_);
  const bool ok = run_tree_sync(trees_[0], 5000ms);
  bool sim = true; (void)blackboard_->get("sim_mode", sim);
  res->success = ok;
  res->message = ok
    ? (sim ? "connected (sim)" : "connected (hardware)")
    : "connect_tree failed — check motor IDs and uart_device param";
  RCLCPP_INFO(get_logger(), "CONNECT %s", ok ? "OK" : "FAILED");
}

void DrivechainNode::on_stop( Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
  if (!drive_active_) { res->success = false; res->message = "node not activated"; return; }
  std::lock_guard<std::mutex> lock(uart_mutex_);
  const bool ok = run_tree_sync(trees_[1], 2000ms);
  res->success = ok;
  res->message = ok ? "stopped" : "stop_tree failed";
  RCLCPP_INFO(get_logger(), "STOP %s", ok ? "OK" : "FAILED");
}

void DrivechainNode::on_drive( DriveService::Request::SharedPtr req, DriveService::Response::SharedPtr res) {
  if (!drive_active_) { res->success = false; res->message = "node not activated"; return; }

  drive_cmd_store_->update(req->motor_commands);

  std::string s;
  for (const auto & c : req->motor_commands)
    s += std::string(s.empty() ? "" : "  ") + "#" + std::to_string(c.motor_id)
       + "→" + std::to_string(c.rpm) + "rpm";
  if (s != last_drive_log_) {
    RCLCPP_DEBUG(get_logger(), "DRIVE: [%s]", s.c_str());
    last_drive_log_ = s;
  }

  res->success = true;
  res->message = "ok";
}

void DrivechainNode::on_set_id( SetIdService::Request::SharedPtr req, SetIdService::Response::SharedPtr res) {
  if (!drive_active_) { res->success = false; res->message = "node not activated"; return; }
  std::lock_guard<std::mutex> lock(uart_mutex_);
  blackboard_->set("target_motor_id", static_cast<int>(req->motor_id));
  blackboard_->set("new_motor_id",    static_cast<int>(req->new_id));
  const bool ok = run_tree_sync(trees_[2], 5000ms);
  res->success = ok;
  res->message = ok
    ? "motor ID changed — disconnect and reconnect to confirm"
    : "set_id_tree failed — ensure only one motor is on the bus";
  RCLCPP_INFO(get_logger(), "SET_ID %d→%d %s", req->motor_id, req->new_id, ok ? "OK" : "FAILED");
}

void DrivechainNode::create_motor_feedback(bool sim_mode) {
  // Build a readable motor summary for the log
  std::vector<MotorDescriptor> motors;
  (void)blackboard_->get("motor_list", motors);
  std::string summary;
  for (const auto & m : motors) {
    if (!summary.empty()) summary += ", ";
    summary += m.name + "#" + std::to_string(m.id);
    if (m.sign < 0) summary += "(inv)";
    if (!m.enabled) summary += "(off)";
  }

  RCLCPP_INFO(get_logger(), "Configured — backend: %s, motors: [%s]", sim_mode ? "sim" : "hardware", summary.c_str());

}

}  // namespace mserve_drivechain

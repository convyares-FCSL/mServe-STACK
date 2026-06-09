#include "include/drivechain_bt_nodes.hpp"

#include <cmath>
#include <functional>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace mserve_drivechain {

// bb_ptr: fetch a typed pointer from the blackboard; nullptr if absent.
template <typename T>
static T * bb_ptr(const BT::NodeConfig & cfg, const std::string & key)
{
  T * p = nullptr;
  (void)cfg.blackboard->get(key, p);
  return p;
}

// bb_get: read a value from the blackboard; return def if the key is absent.
template <typename T>
static T bb_get(const BT::Blackboard::Ptr & bb, const std::string & key, const T & def = T{})
{
  T v = def;
  (void)bb->get(key, v);
  return v;
}

static DriveUart * get_uart(const BT::NodeConfig & cfg)
{
  return bb_ptr<DriveUart>(cfg, "uart");
}

static rclcpp::Logger get_ros_logger(const BT::NodeConfig & cfg)
{
  rclcpp::Logger logger = rclcpp::get_logger("drivechain_bt");
  (void)cfg.blackboard->get("ros_logger", logger);
  return logger;
}

// ==============================================================================
// Conditions
// ==============================================================================

UartOpen::UartOpen(const std::string & name, const BT::NodeConfig & cfg) : BT::ConditionNode(name, cfg) {}

BT::NodeStatus UartOpen::tick()
{
  DriveUart * uart = get_uart(config());
  return (uart && uart->is_open()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// -------------------------------------------------------------------------------

MotorHealthy::MotorHealthy(const std::string & name, const BT::NodeConfig & cfg) : BT::ConditionNode(name, cfg) {}

BT::PortsList MotorHealthy::providedPorts()
{
  return { BT::InputPort<int>("motor_id") };
}

BT::NodeStatus MotorHealthy::tick()
{
  int motor_id = 0;
  getInput("motor_id", motor_id);
  const int left_id = bb_get(config().blackboard, std::string("left_motor_id"), 1);
  const int fault   = bb_get(config().blackboard,
    std::string((motor_id == left_id) ? "left_fault" : "right_fault"), 0);
  return (fault == 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ==============================================================================
// Connect / stop tree actions
// ==============================================================================

OpenUart::OpenUart(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::PortsList OpenUart::providedPorts()
{
  return {
    BT::InputPort<std::string>("device"),
    BT::InputPort<int>("baud"),
  };
}

BT::NodeStatus OpenUart::tick()
{
  DriveUart * uart = get_uart(config());
  if (!uart) return BT::NodeStatus::FAILURE;

  std::string device = "/dev/serial0";
  int baud = 115200;
  getInput("device", device);
  getInput("baud",   baud);

  if (!uart->open(device, baud)) {
    RCLCPP_WARN(get_ros_logger(config()), "OpenUart FAILED: cannot open %s", device.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(get_ros_logger(config()), "OpenUart OK: %s @ %d baud", device.c_str(), baud);
  config().blackboard->set("uart_connected", true);
  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

CloseUart::CloseUart(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::NodeStatus CloseUart::tick()
{
  DriveUart * uart = get_uart(config());
  if (uart) uart->close();
  config().blackboard->set("uart_connected", false);
  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

PingMotor::PingMotor(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::PortsList PingMotor::providedPorts()
{
  return { BT::InputPort<int>("motor_id") };
}

BT::NodeStatus PingMotor::tick()
{
  DriveUart * uart = get_uart(config());
  if (!uart || !uart->is_open()) return BT::NodeStatus::FAILURE;

  int motor_id = 0;
  getInput("motor_id", motor_id);
  const bool ok = uart->ping(static_cast<uint8_t>(motor_id));
  if (!ok) {
    RCLCPP_WARN(get_ros_logger(config()),
      "PingMotor FAILED: motor_id=%d — no response (wrong ID? not powered? RS-485 wiring?)",
      motor_id);
  }
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// -------------------------------------------------------------------------------

SetMotorMode::SetMotorMode(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::PortsList SetMotorMode::providedPorts()
{
  return { BT::InputPort<int>("motor_id"), BT::InputPort<int>("mode") };
}

BT::NodeStatus SetMotorMode::tick()
{
  DriveUart * uart = get_uart(config());
  if (!uart || !uart->is_open()) return BT::NodeStatus::FAILURE;

  int motor_id = 0, mode = 2;
  getInput("motor_id", motor_id);
  getInput("mode",     mode);
  const bool ok = uart->set_mode(static_cast<uint8_t>(motor_id), static_cast<uint8_t>(mode));
  if (!ok) {
    RCLCPP_WARN(get_ros_logger(config()),
      "SetMotorMode FAILED: motor_id=%d mode=%d", motor_id, mode);
  } else {
    RCLCPP_INFO(get_ros_logger(config()),
      "SetMotorMode OK: motor_id=%d mode=%d", motor_id, mode);
  }
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// -------------------------------------------------------------------------------

StopMotor::StopMotor(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::PortsList StopMotor::providedPorts()
{
  return { BT::InputPort<int>("motor_id") };
}

BT::NodeStatus StopMotor::tick()
{
  DriveUart * uart = get_uart(config());
  if (!uart) return BT::NodeStatus::SUCCESS;  // best-effort

  int motor_id = 0;
  getInput("motor_id", motor_id);
  MotorFeedback fb;
  uart->stop(static_cast<uint8_t>(motor_id), fb);
  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

SetMotorId::SetMotorId(const std::string & name, const BT::NodeConfig & cfg) : BT::StatefulActionNode(name, cfg) {}

BT::PortsList SetMotorId::providedPorts()
{
  return { BT::InputPort<int>("motor_id"), BT::InputPort<int>("new_id") };
}

BT::NodeStatus SetMotorId::onStart()
{
  DriveUart * uart = get_uart(config());
  if (!uart || !uart->is_open()) return BT::NodeStatus::FAILURE;

  int motor_id = 0, new_id = 0;
  getInput("motor_id", motor_id);
  getInput("new_id",   new_id);
  result_ = uart->change_id(static_cast<uint8_t>(motor_id), static_cast<uint8_t>(new_id));
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetMotorId::onRunning()
{
  return result_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ==============================================================================
// Drive tree actions
// ==============================================================================

ComputeRpm::ComputeRpm(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::NodeStatus ComputeRpm::tick()
{
  auto * cache = bb_ptr<CmdVelCache>(config(), "cmd_vel_cache");
  auto & bb    = config().blackboard;

  const int    max_rpm    = bb_get(bb, std::string("max_rpm"),            200);
  const int    timeout_ms = bb_get(bb, std::string("cmd_vel_timeout_ms"), 500);
  const double wheel_sep  = bb_get(bb, std::string("wheel_separation"),   0.35);
  const double wheel_rad  = bb_get(bb, std::string("wheel_radius"),       0.08);

  int left_rpm = 0, right_rpm = 0;

  if (cache && cache->age_ms() < static_cast<double>(timeout_ms)) {
    const auto   twist  = cache->latest();
    const double half   = wheel_sep / 2.0;
    const double l_rads = (twist.linear.x - twist.angular.z * half) / wheel_rad;
    const double r_rads = (twist.linear.x + twist.angular.z * half) / wheel_rad;

    const auto to_rpm = [max_rpm](double rads) -> int {
      int rpm = static_cast<int>(std::round(rads * 60.0 / (2.0 * M_PI)));
      return std::clamp(rpm, -max_rpm, max_rpm);
    };
    left_rpm  = to_rpm(l_rads);
    right_rpm = to_rpm(r_rads);
  }
  // Stale / missing cache → write 0/0 (watchdog zero)

  bb->set("left_rpm",  left_rpm);
  bb->set("right_rpm", right_rpm);
  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

SetMotorSpeed::SetMotorSpeed(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::PortsList SetMotorSpeed::providedPorts()
{
  return { BT::InputPort<int>("motor_id"), BT::InputPort<int>("rpm") };
}

BT::NodeStatus SetMotorSpeed::tick()
{
  DriveUart * uart = get_uart(config());
  if (!uart) return BT::NodeStatus::FAILURE;

  int motor_id = 0, rpm = 0;
  getInput("motor_id", motor_id);
  getInput("rpm",      rpm);

  MotorFeedback fb;
  uart->set_speed(static_cast<uint8_t>(motor_id), rpm, fb);

  auto & bb = config().blackboard;
  const int left_id = bb_get(bb, std::string("left_motor_id"), 1);
  const std::string side = (motor_id == left_id) ? "left" : "right";

  constexpr double kRpmToRads = 2.0 * M_PI / 60.0;
  constexpr double kTickToRad = 2.0 * M_PI / 32767.0;
  bb->set(side + "_speed_fb", fb.speed_rpm * kRpmToRads);
  bb->set(side + "_pos_fb",   fb.position  * kTickToRad);
  bb->set(side + "_fault",    fb.fault_code);

  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

PublishWheelFeedback::PublishWheelFeedback(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::NodeStatus PublishWheelFeedback::tick()
{
  using WheelFeedback = mserve_interfaces::msg::WheelFeedback;
  using Fn = std::function<void(const WheelFeedback &)>;

  Fn pub_fn;
  if (!config().blackboard->get("publish_wheel_feedback", pub_fn)) {
    return BT::NodeStatus::SUCCESS;
  }

  auto & bb = config().blackboard;
  WheelFeedback msg;
  msg.left_velocity  = static_cast<float>(bb_get(bb, std::string("left_speed_fb"),  0.0));
  msg.right_velocity = static_cast<float>(bb_get(bb, std::string("right_speed_fb"), 0.0));
  msg.left_position  = static_cast<float>(bb_get(bb, std::string("left_pos_fb"),    0.0));
  msg.right_position = static_cast<float>(bb_get(bb, std::string("right_pos_fb"),   0.0));
  pub_fn(msg);

  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

PublishDriveStatus::PublishDriveStatus(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::NodeStatus PublishDriveStatus::tick()
{
  using DriveStatus = mserve_interfaces::msg::DriveStatus;
  using Fn = std::function<void(const DriveStatus &)>;

  Fn pub_fn;
  if (!config().blackboard->get("publish_drive_status", pub_fn)) {
    return BT::NodeStatus::SUCCESS;
  }

  auto & bb = config().blackboard;
  const bool connected = bb_get(bb, std::string("uart_connected"), false);
  const bool sim_mode  = bb_get(bb, std::string("sim_mode"),       true);

  DriveStatus msg;
  msg.status = connected
    ? (sim_mode ? "connected_sim" : "connected_hw")
    : (sim_mode ? "idle_sim"      : "idle_hw");
  msg.battery_level = 0.0f;
  pub_fn(msg);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mserve_drivechain

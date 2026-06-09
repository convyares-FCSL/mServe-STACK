#include "include/drivechain_bt_nodes.hpp"

#include <cmath>
#include <functional>

namespace mserve_drivechain {

// Helper: fetch typed pointer from blackboard, return nullptr on failure.
template <typename T>
static T * get_bb_ptr(const BT::NodeConfig & cfg, const std::string & key)
{
  T * ptr = nullptr;
  cfg.blackboard->get(key, ptr);
  return ptr;
}

static DriveUart * get_uart(const BT::NodeConfig & cfg)
{
  return get_bb_ptr<DriveUart>(cfg, "uart");
}

// ==============================================================================
// Conditions
// ==============================================================================

UartOpen::UartOpen(const std::string & name, const BT::NodeConfig & cfg)
: BT::ConditionNode(name, cfg) {}

BT::NodeStatus UartOpen::tick()
{
  DriveUart * uart = get_uart(config());
  return (uart && uart->is_open()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// -------------------------------------------------------------------------------

MotorHealthy::MotorHealthy(const std::string & name, const BT::NodeConfig & cfg)
: BT::ConditionNode(name, cfg) {}

BT::PortsList MotorHealthy::providedPorts()
{
  return { BT::InputPort<int>("motor_id") };
}

BT::NodeStatus MotorHealthy::tick()
{
  int motor_id = 0, left_id = 1;
  getInput("motor_id", motor_id);
  config().blackboard->get("left_motor_id", left_id);

  int fault = 0;
  config().blackboard->get((motor_id == left_id) ? "left_fault" : "right_fault", fault);
  return (fault == 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ==============================================================================
// Connect / stop tree actions
// ==============================================================================

OpenUart::OpenUart(const std::string & name, const BT::NodeConfig & cfg)
: BT::SyncActionNode(name, cfg) {}

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

  if (!uart->open(device, baud)) return BT::NodeStatus::FAILURE;

  config().blackboard->set("uart_connected", true);
  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

CloseUart::CloseUart(const std::string & name, const BT::NodeConfig & cfg)
: BT::SyncActionNode(name, cfg) {}

BT::NodeStatus CloseUart::tick()
{
  DriveUart * uart = get_uart(config());
  if (uart) uart->close();
  config().blackboard->set("uart_connected", false);
  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

PingMotor::PingMotor(const std::string & name, const BT::NodeConfig & cfg)
: BT::SyncActionNode(name, cfg) {}

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
  return uart->ping(static_cast<uint8_t>(motor_id))
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// -------------------------------------------------------------------------------

SetMotorMode::SetMotorMode(const std::string & name, const BT::NodeConfig & cfg)
: BT::SyncActionNode(name, cfg) {}

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
  return uart->set_mode(static_cast<uint8_t>(motor_id), static_cast<uint8_t>(mode))
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// -------------------------------------------------------------------------------

StopMotor::StopMotor(const std::string & name, const BT::NodeConfig & cfg)
: BT::SyncActionNode(name, cfg) {}

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

SetMotorId::SetMotorId(const std::string & name, const BT::NodeConfig & cfg)
: BT::StatefulActionNode(name, cfg) {}

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

ComputeRpm::ComputeRpm(const std::string & name, const BT::NodeConfig & cfg)
: BT::SyncActionNode(name, cfg) {}

BT::NodeStatus ComputeRpm::tick()
{
  auto * cache      = get_bb_ptr<CmdVelCache>(config(), "cmd_vel_cache");
  auto * diff_drive = get_bb_ptr<DiffDrive>(config(), "diff_drive");

  int max_rpm = 200, timeout_ms = 500;
  config().blackboard->get("max_rpm",           max_rpm);
  config().blackboard->get("cmd_vel_timeout_ms", timeout_ms);

  int left_rpm = 0, right_rpm = 0;

  if (cache && diff_drive && cache->age_ms() < static_cast<double>(timeout_ms)) {
    const auto twist  = cache->latest();
    const auto speeds = diff_drive->compute(twist.linear.x, twist.angular.z);

    const auto to_rpm = [max_rpm](double rads) -> int {
      int rpm = static_cast<int>(std::round(rads * 60.0 / (2.0 * M_PI)));
      return std::clamp(rpm, -max_rpm, max_rpm);
    };
    left_rpm  = to_rpm(speeds.left);
    right_rpm = to_rpm(speeds.right);
  }
  // If cache is stale or missing: write 0/0 (watchdog zero)

  config().blackboard->set("left_rpm",  left_rpm);
  config().blackboard->set("right_rpm", right_rpm);
  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

SetMotorSpeed::SetMotorSpeed(const std::string & name, const BT::NodeConfig & cfg)
: BT::SyncActionNode(name, cfg) {}

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

  // Write feedback to blackboard using "left_*" / "right_*" keys.
  int left_id = 1;
  config().blackboard->get("left_motor_id", left_id);
  const std::string side = (motor_id == left_id) ? "left" : "right";

  constexpr double kRpmToRads = 2.0 * M_PI / 60.0;
  constexpr double kTickToRad = 2.0 * M_PI / 32767.0;
  config().blackboard->set(side + "_speed_fb", fb.speed_rpm * kRpmToRads);
  config().blackboard->set(side + "_pos_fb",   fb.position  * kTickToRad);
  config().blackboard->set(side + "_fault",    fb.fault_code);

  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

PublishWheelFeedback::PublishWheelFeedback(const std::string & name, const BT::NodeConfig & cfg)
: BT::SyncActionNode(name, cfg) {}

BT::NodeStatus PublishWheelFeedback::tick()
{
  using WheelFeedback = mserve_interfaces::msg::WheelFeedback;
  using Fn = std::function<void(const WheelFeedback &)>;

  Fn pub_fn;
  if (!config().blackboard->get("publish_wheel_feedback", pub_fn)) {
    return BT::NodeStatus::SUCCESS;
  }

  double lv = 0, rv = 0, lp = 0, rp = 0;
  config().blackboard->get("left_speed_fb",  lv);
  config().blackboard->get("right_speed_fb", rv);
  config().blackboard->get("left_pos_fb",    lp);
  config().blackboard->get("right_pos_fb",   rp);

  WheelFeedback msg;
  msg.left_velocity  = static_cast<float>(lv);
  msg.right_velocity = static_cast<float>(rv);
  msg.left_position  = static_cast<float>(lp);
  msg.right_position = static_cast<float>(rp);
  pub_fn(msg);

  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

PublishDriveStatus::PublishDriveStatus(const std::string & name, const BT::NodeConfig & cfg)
: BT::SyncActionNode(name, cfg) {}

BT::NodeStatus PublishDriveStatus::tick()
{
  using DriveStatus = mserve_interfaces::msg::DriveStatus;
  using Fn = std::function<void(const DriveStatus &)>;

  Fn pub_fn;
  if (!config().blackboard->get("publish_drive_status", pub_fn)) {
    return BT::NodeStatus::SUCCESS;
  }

  bool connected = false, sim_mode = true;
  config().blackboard->get("uart_connected", connected);
  config().blackboard->get("sim_mode",       sim_mode);

  DriveStatus msg;
  msg.status = connected
    ? (sim_mode ? "connected_sim" : "connected_hw")
    : (sim_mode ? "idle_sim"      : "idle_hw");
  msg.battery_level = 0.0f;
  pub_fn(msg);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mserve_drivechain

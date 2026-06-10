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

  std::vector<interfaces::msg::MotorState> states;
  (void)config().blackboard->get("motor_states", states);

  for (const auto & s : states) {
    if (s.motor_id == static_cast<uint8_t>(motor_id)) {
      return (s.fault_code == 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::FAILURE;  // motor not found → treat as unhealthy
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

  std::string device = "/dev/ttyAMA0";
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
// N-motor connect / stop
// ==============================================================================

ConnectAllMotors::ConnectAllMotors(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::NodeStatus ConnectAllMotors::tick()
{
  DriveUart * uart = get_uart(config());
  if (!uart || !uart->is_open()) return BT::NodeStatus::FAILURE;

  std::vector<MotorDescriptor> motors;
  (void)config().blackboard->get("motor_list", motors);

  for (const auto & m : motors) {
    if (!m.enabled) continue;

    bool pinged = false;
    for (int attempt = 0; attempt < 3 && !pinged; ++attempt) {
      pinged = uart->ping(m.id);
    }
    if (!pinged) {
      RCLCPP_WARN(get_ros_logger(config()),
        "ConnectAllMotors: motor %s #%d not responding", m.name.c_str(), m.id);
      return BT::NodeStatus::FAILURE;
    }

    if (!uart->set_mode(m.id, 2)) {
      RCLCPP_WARN(get_ros_logger(config()),
        "ConnectAllMotors: motor %s #%d set_mode(2) failed", m.name.c_str(), m.id);
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(get_ros_logger(config()),
      "ConnectAllMotors: motor %s #%d OK", m.name.c_str(), m.id);
  }
  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

StopAllMotors::StopAllMotors(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::NodeStatus StopAllMotors::tick()
{
  DriveUart * uart = get_uart(config());

  std::vector<MotorDescriptor> motors;
  (void)config().blackboard->get("motor_list", motors);

  for (const auto & m : motors) {
    if (!m.enabled) continue;
    if (uart) {
      MotorFeedback fb;
      uart->stop(m.id, fb);
    }
  }
  return BT::NodeStatus::SUCCESS;  // best-effort
}

// ==============================================================================
// Drive tree actions
// ==============================================================================

SetAllMotors::SetAllMotors(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::NodeStatus SetAllMotors::tick()
{
  DriveUart * uart = get_uart(config());
  if (!uart) return BT::NodeStatus::FAILURE;

  auto * store = bb_ptr<DriveCommandStore>(config(), "drive_cmd_store");
  auto & bb    = config().blackboard;

  const int timeout_ms = bb_get(bb, std::string("command_timeout_ms"), 500);

  // Build motor_id → commanded RPM map (empty = zero all motors)
  std::unordered_map<int, int16_t> cmd_map;
  if (store && store->age_ms() < static_cast<double>(timeout_ms)) {
    for (const auto & cmd : store->latest()) {
      cmd_map[cmd.motor_id] = cmd.rpm;
    }
  }

  std::vector<MotorDescriptor> motors;
  (void)bb->get("motor_list", motors);

  std::vector<interfaces::msg::MotorState> states;
  constexpr double kRpmToRads = 2.0 * M_PI / 60.0;
  constexpr double kTickToRad = 2.0 * M_PI / 32767.0;

  for (const auto & m : motors) {
    if (!m.enabled) continue;

    auto it = cmd_map.find(m.id);
    const int16_t raw_rpm  = (it != cmd_map.end()) ? it->second : 0;
    // Apply sign: multiplier compensates for physically reversed motor mounting
    const int16_t send_rpm = static_cast<int16_t>(raw_rpm * m.sign);

    MotorFeedback fb;
    uart->set_speed(m.id, send_rpm, fb);

    interfaces::msg::MotorState state;
    state.motor_id      = m.id;
    state.name          = m.name;
    // Un-flip sign so reported velocity is in the robot's reference frame
    state.velocity_rpm  = static_cast<float>(fb.speed_rpm * m.sign);
    state.velocity_rads = static_cast<float>(fb.speed_rpm * m.sign * kRpmToRads);
    state.position_rad  = static_cast<float>(fb.position  * kTickToRad);
    state.current_a     = fb.current;
    state.temperature_c = fb.temperature;
    state.fault_code    = static_cast<uint8_t>(fb.fault_code);
    states.push_back(state);
  }

  bb->set("motor_states", states);
  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

PublishMotorFeedback::PublishMotorFeedback(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::NodeStatus PublishMotorFeedback::tick()
{
  using DriveMotorFeedback = interfaces::msg::DriveMotorFeedback;
  using Fn = std::function<void(const DriveMotorFeedback &)>;

  Fn pub_fn;
  if (!config().blackboard->get("publish_motor_feedback", pub_fn)) {
    return BT::NodeStatus::SUCCESS;
  }

  std::vector<interfaces::msg::MotorState> states;
  (void)config().blackboard->get("motor_states", states);

  DriveMotorFeedback msg;
  msg.motors = states;
  pub_fn(msg);
  return BT::NodeStatus::SUCCESS;
}

// -------------------------------------------------------------------------------

PublishDriveStatus::PublishDriveStatus(const std::string & name, const BT::NodeConfig & cfg) : BT::SyncActionNode(name, cfg) {}

BT::NodeStatus PublishDriveStatus::tick()
{
  using DriveStatus = interfaces::msg::DriveStatus;
  using Fn = std::function<void(const DriveStatus &)>;

  Fn pub_fn;
  if (!config().blackboard->get("publish_drive_status", pub_fn)) {
    return BT::NodeStatus::SUCCESS;
  }

  DriveUart * uart = get_uart(config());
  auto & bb = config().blackboard;
  const bool connected = bb_get(bb, std::string("uart_connected"), false);
  const bool sim_mode  = bb_get(bb, std::string("sim_mode"),       true);

  DriveStatus msg;
  msg.status = connected
    ? (sim_mode ? "connected_sim" : "connected_hw")
    : (sim_mode ? "idle_sim"      : "idle_hw");
  msg.battery_level = 0.0f;
  msg.board_alive = uart ? uart->board_alive() : false;
  pub_fn(msg);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mserve_drivechain

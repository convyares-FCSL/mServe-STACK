#include "mserve_sensehat/sensehat_node.hpp"

#include <cmath>
#include <optional>

#include <sys/stat.h>

#include <RTIMULib.h>

using namespace std::chrono_literals;

namespace mserve_sensehat {

namespace
{
// Standard gravity, m/s^2 — RTIMULib reports accel in g's (see
// RTIMU::getAccel's comment in RTIMULib's own IMUDrivers/RTIMU.h), but
// sensor_msgs/Imu.msg requires m/s^2.
constexpr double kGravity = 9.80665;
constexpr double kRadToDeg = 180.0 / M_PI;

std::array<uint8_t, LedMatrix::kSize> iconX()
{
  return {0b10000001, 0b01000010, 0b00100100, 0b00011000,
    0b00011000, 0b00100100, 0b01000010, 0b10000001};
}

std::array<uint8_t, LedMatrix::kSize> iconO()
{
  return {0b00111100, 0b01000010, 0b10000001, 0b10000001,
    0b10000001, 0b10000001, 0b01000010, 0b00111100};
}

std::optional<JoyKey> keyFromName(const std::string & name)
{
  if (name == "up") {return JoyKey::Up;}
  if (name == "down") {return JoyKey::Down;}
  if (name == "left") {return JoyKey::Left;}
  if (name == "right") {return JoyKey::Right;}
  if (name == "center") {return JoyKey::Center;}
  return std::nullopt;
}
}  // namespace

SensehatNode::SensehatNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mserve_sensehat", options)
{
  declareParams();
  loadParams();
  loadButtonActions();
  buildButtonDispatch();

  // RTIMUSettings writes/reads a small .ini cache of whatever discoverIMU()
  // finds on first run (chip type + I2C address) — needs a writable
  // directory, created here since it won't exist on a fresh container.
  ::mkdir(imu_settings_dir_.c_str(), 0755);
  imu_settings_ = std::make_unique<RTIMUSettings>(imu_settings_dir_.c_str(), "RTIMULib");
  imu_.reset(RTIMU::createIMU(imu_settings_.get()));
  if (imu_ && imu_->IMUType() != RTIMU_TYPE_NULL && imu_->IMUInit()) {
    imu_available_ = true;
    RCLCPP_INFO(get_logger(), "IMU found: %s", imu_->IMUName());
  } else {
    RCLCPP_WARN(get_logger(), "no IMU found on I2C bus — publishing nothing on '%s'",
      topic_imu_.c_str());
  }

  // Pressure/humidity discovery is independent of the IMU chip — same
  // RTIMUSettings instance, same AUTODISCOVER-by-default mechanism.
  pressure_.reset(RTPressure::createPressure(imu_settings_.get()));
  if (pressure_ && pressure_->pressureInit()) {
    pressure_available_ = true;
  } else {
    pressure_.reset();
  }
  humidity_.reset(RTHumidity::createHumidity(imu_settings_.get()));
  if (humidity_ && humidity_->humidityInit()) {
    humidity_available_ = true;
  } else {
    humidity_.reset();
  }
  RCLCPP_INFO(
    get_logger(), "pressure sensor: %s, humidity sensor: %s",
    pressure_available_ ? "yes" : "no", humidity_available_ ? "yes" : "no");

  led_available_ = led_.open();
  if (!led_available_) {
    RCLCPP_WARN(get_logger(), "LED matrix not available — status icon disabled");
  }

  joystick_available_ = joystick_.open(joystick_device_name_match_);
  if (!joystick_available_) {
    RCLCPP_WARN(get_logger(), "joystick not available — button actions disabled");
  }

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(topic_imu_, rclcpp::SensorDataQoS());
  status_pub_ = create_publisher<interfaces::msg::SensehatStatus>(topic_status_, rclcpp::QoS(10));
  drivechain_status_sub_ = create_subscription<interfaces::msg::DriveStatus>(
    topic_drivechain_status_, rclcpp::QoS(10),
    std::bind(&SensehatNode::onDrivechainStatus, this, std::placeholders::_1));
  connect_client_ = create_client<std_srvs::srv::Trigger>(service_connect_);
  set_online_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/set_online",
    std::bind(&SensehatNode::onSetOnline, this, std::placeholders::_1, std::placeholders::_2));

  if (imu_available_ && imu_publish_hz_ > 0.0) {
    const auto period = std::chrono::duration<double>(1.0 / imu_publish_hz_);
    imu_poll_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&SensehatNode::onImuPollTimer, this));
  }
  if (joystick_available_ && joystick_poll_hz_ > 0.0) {
    const auto period = std::chrono::duration<double>(1.0 / joystick_poll_hz_);
    joystick_poll_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&SensehatNode::onJoystickPollTimer, this));
  }
  if (status_publish_hz_ > 0.0) {
    const auto period = std::chrono::duration<double>(1.0 / status_publish_hz_);
    status_publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&SensehatNode::onStatusPublishTimer, this));
  }

  renderStatusIcon();

  RCLCPP_INFO(
    get_logger(), "mserve_sensehat ready: imu=%s led=%s joystick=%s",
    imu_available_ ? "yes" : "no", led_available_ ? "yes" : "no",
    joystick_available_ ? "yes" : "no");
}

SensehatNode::~SensehatNode()
{
  led_.close();
  joystick_.close();
  // imu_/pressure_/humidity_ must be destroyed before imu_settings_ (they
  // keep a raw pointer back to the settings that constructed them) —
  // unique_ptr member destruction order (declaration order, reversed)
  // already guarantees this since they're declared after imu_settings_ in
  // the header.
}

void SensehatNode::buildButtonDispatch()
{
  const std::map<std::string, std::function<void()>> action_handlers = {
    {"connect", [this] {callConnect();}},
  };

  for (const auto & [name, action] : button_actions_) {
    const auto key = keyFromName(name);
    if (!key) {
      RCLCPP_WARN(
        get_logger(), "button_actions: '%s' is not a valid key (up/down/left/right/center)",
        name.c_str());
      continue;
    }
    const auto handler_it = action_handlers.find(action);
    if (handler_it == action_handlers.end()) {
      RCLCPP_WARN(
        get_logger(), "button_actions: unknown action '%s' for key '%s' — ignored",
        action.c_str(), name.c_str());
      continue;
    }
    key_to_handler_[*key] = handler_it->second;
  }
}

void SensehatNode::onImuPollTimer()
{
  if (!imu_->IMURead()) {
    return;
  }
  const RTIMU_DATA & data = imu_->getIMUData();

  sensor_msgs::msg::Imu msg;
  msg.header.stamp = now();
  msg.header.frame_id = imu_frame_id_;

  if (data.fusionQPoseValid) {
    msg.orientation.w = data.fusionQPose.scalar();
    msg.orientation.x = data.fusionQPose.x();
    msg.orientation.y = data.fusionQPose.y();
    msg.orientation.z = data.fusionQPose.z();
  } else {
    msg.orientation_covariance[0] = -1.0;  // standard "orientation not available" marker
  }

  msg.angular_velocity.x = data.gyro.x();
  msg.angular_velocity.y = data.gyro.y();
  msg.angular_velocity.z = data.gyro.z();

  msg.linear_acceleration.x = data.accel.x() * kGravity;
  msg.linear_acceleration.y = data.accel.y() * kGravity;
  msg.linear_acceleration.z = data.accel.z() * kGravity;

  imu_pub_->publish(msg);

  // Cached for onStatusPublishTimer — native/human-friendly units (g, deg/s,
  // uT) rather than the SI units above, see SensehatStatus.msg's comment.
  imu_data_valid_ = true;
  last_accel_g_[0] = data.accel.x();
  last_accel_g_[1] = data.accel.y();
  last_accel_g_[2] = data.accel.z();
  last_gyro_dps_[0] = data.gyro.x() * kRadToDeg;
  last_gyro_dps_[1] = data.gyro.y() * kRadToDeg;
  last_gyro_dps_[2] = data.gyro.z() * kRadToDeg;
  last_mag_ut_[0] = data.compass.x();
  last_mag_ut_[1] = data.compass.y();
  last_mag_ut_[2] = data.compass.z();
  if (data.fusionPoseValid) {
    // fusionPose.z() is yaw in radians, [-pi, pi] — normalize to a 0-360
    // compass heading. Already tilt/mag-compensated by RTIMULib's fusion
    // filter (accel+gyro+compass), not a raw compass angle.
    double heading = std::fmod(data.fusionPose.z() * kRadToDeg, 360.0);
    if (heading < 0.0) {
      heading += 360.0;
    }
    last_heading_deg_ = heading;
  }
}

void SensehatNode::onJoystickPollTimer()
{
  for (const JoyEvent & ev : joystick_.poll()) {
    switch (ev.key) {
      case JoyKey::Up: joy_up_ = ev.pressed; break;
      case JoyKey::Down: joy_down_ = ev.pressed; break;
      case JoyKey::Left: joy_left_ = ev.pressed; break;
      case JoyKey::Right: joy_right_ = ev.pressed; break;
      case JoyKey::Center: joy_center_ = ev.pressed; break;
    }
    if (!ev.pressed || !online_) {
      continue;
    }
    const auto handler_it = key_to_handler_.find(ev.key);
    if (handler_it != key_to_handler_.end()) {
      handler_it->second();
    }
  }
}

void SensehatNode::onStatusPublishTimer()
{
  interfaces::msg::SensehatStatus msg;
  msg.online = online_;
  msg.imu_available = imu_available_;
  msg.pressure_available = pressure_available_;
  msg.humidity_available = humidity_available_;
  msg.led_available = led_available_;
  msg.joystick_available = joystick_available_;

  if (imu_data_valid_) {
    msg.accel_g.x = last_accel_g_[0];
    msg.accel_g.y = last_accel_g_[1];
    msg.accel_g.z = last_accel_g_[2];
    msg.gyro_dps.x = last_gyro_dps_[0];
    msg.gyro_dps.y = last_gyro_dps_[1];
    msg.gyro_dps.z = last_gyro_dps_[2];
    msg.mag_ut.x = last_mag_ut_[0];
    msg.mag_ut.y = last_mag_ut_[1];
    msg.mag_ut.z = last_mag_ut_[2];
    msg.heading_deg = static_cast<float>(last_heading_deg_);
  }
  if (imu_available_) {
    msg.compass_calibrated = imu_->getCompassCalibrationValid();
    msg.accel_calibrated = imu_->getAccelCalibrationValid();
  }

  if (pressure_available_) {
    RTIMU_DATA pressure_data{};
    if (pressure_->pressureRead(pressure_data)) {
      if (pressure_data.pressureValid) {
        msg.pressure_hpa = static_cast<float>(pressure_data.pressure);
      }
      if (pressure_data.temperatureValid) {
        msg.temperature_c = static_cast<float>(pressure_data.temperature);
      }
    }
  }
  if (humidity_available_) {
    RTIMU_DATA humidity_data{};
    if (humidity_->humidityRead(humidity_data)) {
      if (humidity_data.humidityValid) {
        msg.humidity_percent = static_cast<float>(humidity_data.humidity);
      }
      // Prefer the pressure chip's temperature reading if we already have
      // one (set above) — only fall back to the humidity chip's if not.
      if (humidity_data.temperatureValid && msg.temperature_c == 0.0f) {
        msg.temperature_c = static_cast<float>(humidity_data.temperature);
      }
    }
  }

  msg.joy_up = joy_up_;
  msg.joy_down = joy_down_;
  msg.joy_left = joy_left_;
  msg.joy_right = joy_right_;
  msg.joy_center = joy_center_;

  status_pub_->publish(msg);
}

void SensehatNode::onDrivechainStatus(interfaces::msg::DriveStatus::SharedPtr msg)
{
  // Same prefix check as mserve_display::screens.cpp's Menu screen — msg
  // status is one of "disconnected"/"connecting"/"connected"/"error: ...".
  bool connected = msg->status.rfind("connected", 0) == 0;
  if (connected != drivechain_connected_) {
    drivechain_connected_ = connected;
    renderStatusIcon();
  }
}

void SensehatNode::onSetOnline(
  std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  online_ = request->data;
  RCLCPP_INFO(get_logger(), "online -> %s", online_ ? "true" : "false");
  response->success = true;
  response->message = online_ ? "online" : "offline (button_actions disabled, sensors still read)";
}

void SensehatNode::renderStatusIcon()
{
  if (!led_available_) {
    return;
  }
  if (drivechain_connected_) {
    led_.drawBitmap(iconO(), LedMatrix::rgb565(0, 200, 0));
  } else {
    led_.drawBitmap(iconX(), LedMatrix::rgb565(200, 0, 0));
  }
  led_.present();
}

void SensehatNode::callConnect()
{
  if (connect_in_flight_) {
    return;
  }
  if (!connect_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "connect: service '%s' unavailable", service_connect_.c_str());
    return;
  }
  connect_in_flight_ = true;
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  connect_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      connect_in_flight_ = false;
      const auto resp = future.get();
      RCLCPP_INFO(
        get_logger(), "connect: success=%d message='%s'", resp->success, resp->message.c_str());
    });
}

}  // namespace mserve_sensehat

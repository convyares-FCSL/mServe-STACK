#pragma once

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <interfaces/msg/drive_status.hpp>
#include <interfaces/msg/sensehat_status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mserve_sensehat/joystick_input.hpp"
#include "mserve_sensehat/led_matrix.hpp"

class RTIMUSettings;
class RTIMU;
class RTPressure;
class RTHumidity;

namespace mserve_sensehat {

// Plain rclcpp::Node, not lifecycle-managed — same reasoning as
// mserve_display/mserve_joystick: the kernel already owns the framebuffer/
// input-event/I2C devices, this node just reads/writes them, no hardware
// "connect" step of its own.
class SensehatNode : public rclcpp::Node
{
public:
  explicit SensehatNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SensehatNode() override;

private:
  void declareParams();
  void loadParams();
  void loadButtonActions();
  void buildButtonDispatch();

  void onImuPollTimer();
  void onJoystickPollTimer();
  void onStatusPublishTimer();
  void onDrivechainStatus(interfaces::msg::DriveStatus::SharedPtr msg);
  void onSetOnline(
    std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);

  void callConnect();
  void renderStatusIcon();

  // IMU (RTIMULib) — raw pointers match RTIMULib's own ownership model
  // (RTIMU::createIMU returns a heap object that outlives, and holds a
  // pointer back to, the RTIMUSettings that constructed it).
  std::unique_ptr<RTIMUSettings> imu_settings_;
  std::unique_ptr<RTIMU> imu_;
  bool imu_available_ = false;
  bool imu_data_valid_ = false;
  double last_accel_g_[3] = {0.0, 0.0, 0.0};
  double last_gyro_dps_[3] = {0.0, 0.0, 0.0};
  double last_mag_ut_[3] = {0.0, 0.0, 0.0};
  double last_heading_deg_ = 0.0;

  std::unique_ptr<RTPressure> pressure_;
  bool pressure_available_ = false;
  std::unique_ptr<RTHumidity> humidity_;
  bool humidity_available_ = false;

  LedMatrix led_;
  bool led_available_ = false;

  JoystickInput joystick_;
  bool joystick_available_ = false;
  bool joy_up_ = false;
  bool joy_down_ = false;
  bool joy_left_ = false;
  bool joy_right_ = false;
  bool joy_center_ = false;

  bool drivechain_connected_ = false;
  bool online_ = true;

  // Name (up/down/left/right/center) -> action (e.g. "connect") from
  // button_actions.* params, and the resolved name -> handler dispatch —
  // same two-stage pattern as mserve_joystick's buildButtonDispatch, so the
  // action this button performs can be reconfigured/reassigned the same
  // way (mserve_params.yaml), not hardcoded to "center always connects".
  std::map<std::string, std::string> button_actions_;
  std::map<JoyKey, std::function<void()>> key_to_handler_;

  // Publisher / subscription / client / service
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<interfaces::msg::SensehatStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<interfaces::msg::DriveStatus>::SharedPtr drivechain_status_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr connect_client_;
  bool connect_in_flight_ = false;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_online_srv_;

  // Timers
  rclcpp::TimerBase::SharedPtr imu_poll_timer_;
  rclcpp::TimerBase::SharedPtr joystick_poll_timer_;
  rclcpp::TimerBase::SharedPtr status_publish_timer_;

  // Params
  std::string imu_frame_id_;
  double imu_publish_hz_ = 20.0;
  std::string imu_settings_dir_;
  double joystick_poll_hz_ = 20.0;
  std::string joystick_device_name_match_;
  double status_publish_hz_ = 5.0;
  std::string topic_imu_;
  std::string topic_status_;
  std::string topic_drivechain_status_;
  std::string service_connect_;
};

}  // namespace mserve_sensehat

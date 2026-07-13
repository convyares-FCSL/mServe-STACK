#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <sl_lidar.h>

namespace mserve_lidar {

class LidarNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit LidarNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LidarNode() override;

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  // Params (lidar_params.cpp)
  void declare_params();
  void load_params();
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> &);
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string device_;
  int baudrate_ = 460800;
  std::string frame_id_;
  std::string scan_mode_;   // empty = lidar's typical/default mode
  bool inverted_ = false;

  bool connect();
  bool check_device_info();
  bool check_health();
  bool set_scan_mode();
  void disconnect();

  // grabScanDataHq() blocks until a full revolution is ready, so this runs on
  // its own thread rather than a fixed-rate timer — same reasoning as
  // mserve_camera's capture_loop() around V4l2CameraDevice::capture().
  void capture_loop();
  void publish_scan(
    const std::vector<sl_lidar_response_measurement_node_hq_t> & nodes,
    size_t first, size_t last, rclcpp::Time stamp, double scan_duration);

  sl::IChannel * channel_ = nullptr;
  sl::ILidarDriver * driver_ = nullptr;
  float max_distance_m_ = 12.0f;  // overwritten by the scan mode actually negotiated on activate

  std::thread       capture_thread_;
  std::atomic<bool> capturing_{false};

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

}  // namespace mserve_lidar

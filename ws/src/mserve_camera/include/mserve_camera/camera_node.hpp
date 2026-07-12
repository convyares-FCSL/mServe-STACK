#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <v4l2_camera/v4l2_camera_device.hpp>

namespace mserve_camera {

class CameraNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode() override;

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  // Params (camera_params.cpp)
  void declare_params();
  void load_params();
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> &);
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string device_;
  int width_  = 640;
  int height_ = 480;
  std::string frame_id_;

  // Capture loop — V4l2CameraDevice::capture() blocks until a frame is ready,
  // so this runs on its own thread rather than a fixed-rate timer (matches
  // upstream v4l2_camera_node's own capture_thread_/canceled_ pattern).
  void capture_loop();

  std::unique_ptr<v4l2_camera::V4l2CameraDevice> camera_;
  std::thread       capture_thread_;
  std::atomic<bool> capturing_{false};

  sensor_msgs::msg::CameraInfo camera_info_;  // uncalibrated placeholder — see camera_params.cpp

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr      image_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
};

}  // namespace mserve_camera

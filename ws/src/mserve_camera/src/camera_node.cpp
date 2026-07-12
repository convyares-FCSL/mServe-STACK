#include "mserve_camera/camera_node.hpp"

#include <linux/videodev2.h>
#include <v4l2_camera/fourcc.hpp>

namespace mserve_camera {

// ==============================================================================
// Construction / destruction
// ==============================================================================

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("mserve_camera", options)
{
  declare_params();
  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) { return on_parameters(p); });
}

CameraNode::~CameraNode() {
  capturing_ = false;
  if (capture_thread_.joinable()) capture_thread_.join();
}

// ==============================================================================
// Lifecycle
// ==============================================================================

CameraNode::CallbackReturn CameraNode::on_configure(const rclcpp_lifecycle::State &) {
  load_params();

  camera_ = std::make_unique<v4l2_camera::V4l2CameraDevice>(device_);
  if (!camera_->open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open %s", device_.c_str());
    camera_.reset();
    return CallbackReturn::FAILURE;
  }

  v4l2_camera::PixelFormat requested;
  requested.width = static_cast<unsigned>(width_);
  requested.height = static_cast<unsigned>(height_);
  requested.pixelFormat = V4L2_PIX_FMT_YUYV;
  if (!camera_->requestDataFormat(requested)) {
    RCLCPP_ERROR(get_logger(), "Failed to set %dx%d YUYV on %s", width_, height_, device_.c_str());
    camera_.reset();
    return CallbackReturn::FAILURE;
  }

  // The driver may silently pick the nearest supported size — reflect
  // whatever it actually settled on, not just what we asked for.
  const auto & actual = camera_->getCurrentDataFormat();
  width_  = static_cast<int>(actual.width);
  height_ = static_cast<int>(actual.height);
  camera_info_.width  = actual.width;
  camera_info_.height = actual.height;

  // Reliable, not SensorDataQoS (best-effort) — web_video_server hardcodes a
  // reliable subscription with no per-stream override, so best-effort here
  // silently drops every frame for it ("offering incompatible QoS" warning,
  // no image ever arrives). Fine at this resolution/rate over localhost.
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", rclcpp::QoS(10));
  info_pub_  = create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(), "Configured — %s, %ux%u %s, frame_id=%s",
    device_.c_str(), actual.width, actual.height,
    v4l2_camera::FourCC::toString(actual.pixelFormat).c_str(), frame_id_.c_str());

  return CallbackReturn::SUCCESS;
}

CameraNode::CallbackReturn CameraNode::on_activate(const rclcpp_lifecycle::State &) {
  if (!camera_ || !camera_->start()) {
    RCLCPP_ERROR(get_logger(), "Failed to start capture on %s", device_.c_str());
    return CallbackReturn::FAILURE;
  }

  image_pub_->on_activate();
  info_pub_->on_activate();

  capturing_ = true;
  capture_thread_ = std::thread(&CameraNode::capture_loop, this);

  RCLCPP_INFO(get_logger(), "Activated — publishing camera/image_raw + camera/camera_info");
  return CallbackReturn::SUCCESS;
}

CameraNode::CallbackReturn CameraNode::on_deactivate(const rclcpp_lifecycle::State &) {
  capturing_ = false;
  if (capture_thread_.joinable()) capture_thread_.join();
  if (camera_) camera_->stop();

  image_pub_->on_deactivate();
  info_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

CameraNode::CallbackReturn CameraNode::on_cleanup(const rclcpp_lifecycle::State &) {
  capturing_ = false;
  if (capture_thread_.joinable()) capture_thread_.join();
  camera_.reset();
  image_pub_.reset();
  info_pub_.reset();
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

CameraNode::CallbackReturn CameraNode::on_shutdown(const rclcpp_lifecycle::State &) {
  capturing_ = false;
  if (capture_thread_.joinable()) capture_thread_.join();
  if (camera_) { camera_->stop(); camera_.reset(); }
  RCLCPP_INFO(get_logger(), "Shutdown");
  return CallbackReturn::SUCCESS;
}

// ==============================================================================
// Capture loop
// ==============================================================================

void CameraNode::capture_loop() {
  while (capturing_.load()) {
    sensor_msgs::msg::Image::UniquePtr image;
    try {
      image = camera_->capture();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Capture error: %s", e.what());
      continue;
    }
    if (!image) continue;

    image->header.stamp = now();
    image->header.frame_id = frame_id_;

    auto info = camera_info_;
    info.header.stamp = image->header.stamp;

    if (image_pub_->is_activated()) image_pub_->publish(std::move(image));
    if (info_pub_->is_activated())  info_pub_->publish(info);
  }
}

}  // namespace mserve_camera

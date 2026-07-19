#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <opencv2/core/mat.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
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
  int jpeg_quality_ = 80;
  bool flip_180_ = true;  // physical mount is upside down — see camera_params.cpp
  // Ceiling on capture_loop()'s own iteration rate — see capture_loop()'s
  // comment for why this can't just rely on V4l2CameraDevice::capture()
  // blocking naturally. Default sits comfortably above the ~12.6Hz this
  // camera actually sustains (see camera_limits.hpp's kTargetFps comment),
  // so it costs nothing in normal operation.
  double capture_rate_hz_ = 15.0;

  // Capture loop — V4l2CameraDevice::capture() *should* block until a frame
  // is ready, so this runs on its own thread rather than a fixed-rate timer
  // (matches upstream v4l2_camera_node's own capture_thread_/canceled_
  // pattern) — but confirmed on real hardware that it doesn't always: under
  // a sustained device failure it can return (or fail to return) far faster
  // than any real camera frame rate, so capture_rate_hz_ paces this loop
  // independent of that call's own behavior rather than trusting it.
  void capture_loop();

  // VIDIOC_S_PARM (frame interval) — V4l2CameraDevice has no wrapper for it,
  // so this reaches the device directly via a second, independent open() of
  // the same node. See camera_limits.hpp's kTargetFps — as of 2026-07-13 this
  // is confirmed NOT to actually change the streamed rate; kept only because
  // it's a harmless no-op on failure, pending a real fix.
  bool request_frame_rate(int fps);

  // YUYV -> BGR (+ 180-degree rotate if flip_180_) done once per frame in
  // capture_loop(), shared by both outputs below — rotating the packed YUYV
  // bytes directly instead would reverse column order, which silently swaps
  // the U/V byte roles for even widths (wrong colors, not just a crash), so
  // the flip only ever happens after conversion to BGR's one-byte-per-
  // channel layout, where a flip is unambiguous.
  cv::Mat convert_and_flip(const sensor_msgs::msg::Image & yuyv_image);

  // BGR -> JPEG via OpenCV directly (cv::imencode) — hand-rolled rather than
  // pulled in via image_transport's plugin system, because
  // image_transport::Publisher wraps a plain rclcpp::Publisher, not a
  // LifecyclePublisher, which would silently break the "deactivate stops all
  // publishing" guarantee the rest of this node relies on. Returns nullptr
  // if the encode fails (never expected to under normal operation).
  sensor_msgs::msg::CompressedImage::UniquePtr encode_compressed(
    const cv::Mat & bgr, const std_msgs::msg::Header & header);

  std::unique_ptr<v4l2_camera::V4l2CameraDevice> camera_;
  std::thread       capture_thread_;
  std::atomic<bool> capturing_{false};

  sensor_msgs::msg::CameraInfo camera_info_;  // uncalibrated placeholder — see camera_params.cpp

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr           image_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr      info_pub_;
};

}  // namespace mserve_camera

#include "mserve_camera/camera_node.hpp"
#include "mserve_camera/camera_limits.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/videodev2.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
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

  if (!request_frame_rate(kTargetFps)) {
    RCLCPP_WARN(get_logger(), "Could not request %d fps — camera may run at whatever "
      "frame interval was last active", kTargetFps);
  }

  // Reliable, not SensorDataQoS (best-effort) — web_video_server hardcodes a
  // reliable subscription with no per-stream override, so best-effort here
  // silently drops every frame for it ("offering incompatible QoS" warning,
  // no image ever arrives). Fine at this resolution/rate over localhost.
  image_pub_      = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", rclcpp::QoS(10));
  compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
    "camera/image_raw/compressed", rclcpp::QoS(10));
  info_pub_       = create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", rclcpp::QoS(10));

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
  compressed_pub_->on_activate();
  info_pub_->on_activate();

  capturing_ = true;
  capture_thread_ = std::thread(&CameraNode::capture_loop, this);

  RCLCPP_INFO(get_logger(), "Activated — publishing camera/image_raw (+ /compressed) + camera/camera_info");
  return CallbackReturn::SUCCESS;
}

CameraNode::CallbackReturn CameraNode::on_deactivate(const rclcpp_lifecycle::State &) {
  capturing_ = false;
  if (capture_thread_.joinable()) capture_thread_.join();
  if (camera_) camera_->stop();

  image_pub_->on_deactivate();
  compressed_pub_->on_deactivate();
  info_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

CameraNode::CallbackReturn CameraNode::on_cleanup(const rclcpp_lifecycle::State &) {
  capturing_ = false;
  if (capture_thread_.joinable()) capture_thread_.join();
  camera_.reset();
  image_pub_.reset();
  compressed_pub_.reset();
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
// Frame rate
// ==============================================================================

bool CameraNode::request_frame_rate(int fps) {
  // NOTE: verified 2026-07-13 that this does NOT actually change the
  // streamed rate on this hardware/driver, despite the ioctl reporting
  // success — see camera_limits.hpp's kTargetFps comment. Kept as a no-op
  // pending a real fix; a second, independent open() of the device node
  // apparently doesn't share frame-interval negotiation with the fd that
  // actually calls camera_->start() (VIDIOC_STREAMON) on this driver.
  const int fd = ::open(device_.c_str(), O_RDWR);
  if (fd < 0) {
    RCLCPP_WARN(get_logger(), "Could not reopen %s to set frame interval: %s",
      device_.c_str(), std::strerror(errno));
    return false;
  }

  v4l2_streamparm streamparm{};
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  streamparm.parm.capture.timeperframe.numerator = 1;
  streamparm.parm.capture.timeperframe.denominator = static_cast<uint32_t>(fps);

  const bool ok = ::ioctl(fd, VIDIOC_S_PARM, &streamparm) >= 0;
  if (!ok) {
    RCLCPP_WARN(get_logger(), "VIDIOC_S_PARM failed (requesting %d fps): %s",
      fps, std::strerror(errno));
  }
  ::close(fd);
  return ok;
}

// ==============================================================================
// Compressed encode
// ==============================================================================

sensor_msgs::msg::CompressedImage::UniquePtr CameraNode::encode_compressed(
  const sensor_msgs::msg::Image & image)
{
  // Wraps the existing buffer (no copy) — safe because this only reads from
  // it, and the caller still owns `image` at this point.
  const cv::Mat yuyv(
    static_cast<int>(image.height), static_cast<int>(image.width), CV_8UC2,
    const_cast<uint8_t *>(image.data.data()), image.step);

  cv::Mat bgr;
  cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);

  std::vector<uchar> jpeg_buf;
  const std::vector<int> encode_params{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
  if (!cv::imencode(".jpg", bgr, jpeg_buf, encode_params)) {
    RCLCPP_ERROR(get_logger(), "cv::imencode failed for a %ux%u frame", image.width, image.height);
    return nullptr;
  }

  auto compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
  compressed->header = image.header;
  compressed->format = "jpeg";
  compressed->data.assign(jpeg_buf.begin(), jpeg_buf.end());
  return compressed;
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

    // Encode from *image before it's moved out below — publish() takes ownership.
    if (compressed_pub_->is_activated()) {
      if (auto compressed = encode_compressed(*image)) {
        compressed_pub_->publish(std::move(compressed));
      }
    }

    if (image_pub_->is_activated()) image_pub_->publish(std::move(image));
    if (info_pub_->is_activated())  info_pub_->publish(info);
  }
}

}  // namespace mserve_camera

#include "mserve_camera/camera_node.hpp"
#include "mserve_camera/camera_limits.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <thread>
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
  // Verified on real hardware: this does NOT actually change the streamed
  // rate on this driver, despite the ioctl reporting success — see
  // camera_limits.hpp's kTargetFps comment. Kept as a harmless no-op
  // pending a real fix.
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

cv::Mat CameraNode::convert_and_flip(const sensor_msgs::msg::Image & yuyv_image)
{
  // Wraps the existing buffer (no copy) — safe because this only reads from
  // it, and the caller still owns `yuyv_image` at this point.
  const cv::Mat yuyv(
    static_cast<int>(yuyv_image.height), static_cast<int>(yuyv_image.width), CV_8UC2,
    const_cast<uint8_t *>(yuyv_image.data.data()), yuyv_image.step);

  cv::Mat bgr;
  cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);
  if (flip_180_) {
    cv::rotate(bgr, bgr, cv::ROTATE_180);
  }
  return bgr;
}

sensor_msgs::msg::CompressedImage::UniquePtr CameraNode::encode_compressed(
  const cv::Mat & bgr, const std_msgs::msg::Header & header)
{
  std::vector<uchar> jpeg_buf;
  const std::vector<int> encode_params{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
  if (!cv::imencode(".jpg", bgr, jpeg_buf, encode_params)) {
    RCLCPP_ERROR(get_logger(), "cv::imencode failed for a %dx%d frame", bgr.cols, bgr.rows);
    return nullptr;
  }

  auto compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
  compressed->header = header;
  compressed->format = "jpeg";
  compressed->data.assign(jpeg_buf.begin(), jpeg_buf.end());
  return compressed;
}

// ==============================================================================
// Capture loop
// ==============================================================================

void CameraNode::capture_loop() {
  // Paced to a defined cycle rather than trusting V4l2CameraDevice::
  // capture() to naturally rate-limit this loop by blocking. It normally
  // does — but confirmed on real hardware that under a
  // sustained device failure, a *single* call can internally spin logging
  // "Error dequeueing buffer" far faster than any camera frame rate
  // (millions of lines / a full CPU core pinned within minutes), without
  // ever throwing or returning null — so a retry-backoff wrapped around the
  // call never even regains control to run. That failure mode can't be
  // fixed from here (it's inside the apt-packaged v4l2_camera library) —
  // but pacing this loop to a fixed cycle at least caps the case where
  // capture() *does* return control (fast success, fast failure, or a
  // bounded number of internal retries), which is what actually matters
  // day to day: a cheap 640x480/~12.6Hz webcam has no business anywhere
  // near a full CPU core.
  const auto target_period = std::chrono::duration<double>(1.0 / capture_rate_hz_);
  int consecutive_failures = 0;
  auto last_health_log = std::chrono::steady_clock::now();

  while (capturing_.load()) {
    const auto cycle_start = std::chrono::steady_clock::now();

    sensor_msgs::msg::Image::UniquePtr raw_yuyv;
    try {
      raw_yuyv = camera_->capture();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Capture error: %s", e.what());
      raw_yuyv.reset();
    }

    if (!raw_yuyv) {
      ++consecutive_failures;
      // v4l2_camera's own capture() already logs the underlying V4L2 error
      // on every failure — this is a periodic summary on top of that, not
      // a replacement, rate-limited so a sustained outage stays readable
      // instead of spamming once per cycle.
      if (cycle_start - last_health_log >= std::chrono::seconds(5)) {
        RCLCPP_WARN(
          get_logger(), "capture_loop: %d consecutive failures, device may be gone",
          consecutive_failures);
        last_health_log = cycle_start;
      }
    } else {
      consecutive_failures = 0;
    }

    if (raw_yuyv) {
      raw_yuyv->header.stamp = now();
      raw_yuyv->header.frame_id = frame_id_;

      auto info = camera_info_;
      info.header.stamp = raw_yuyv->header.stamp;

      // Gated on actual subscriber count, not just lifecycle-active state.
      // In normal operation *nothing* subscribes to image_raw/compressed —
      // both web UI pages stream image_raw itself, and web_video_server
      // does its own MJPEG transcode from that — and encode_compressed()'s
      // full JPEG encode is the most expensive single step in this loop,
      // so encoding without a subscriber is pure waste.
      const bool want_compressed =
        compressed_pub_->is_activated() && compressed_pub_->get_subscription_count() > 0;
      const bool want_raw =
        image_pub_->is_activated() && image_pub_->get_subscription_count() > 0;

      if (want_compressed || want_raw) {
        // Converted (and flip_180_-rotated) once, shared by both outputs
        // below — see convert_and_flip()'s comment for why the flip
        // happens here and not on the raw YUYV bytes.
        const cv::Mat bgr = convert_and_flip(*raw_yuyv);

        if (want_compressed) {
          if (auto compressed = encode_compressed(bgr, raw_yuyv->header)) {
            compressed_pub_->publish(std::move(compressed));
          }
        }

        if (want_raw) {
          auto bgr_image = std::make_unique<sensor_msgs::msg::Image>();
          bgr_image->header = raw_yuyv->header;
          bgr_image->height = static_cast<uint32_t>(bgr.rows);
          bgr_image->width  = static_cast<uint32_t>(bgr.cols);
          bgr_image->encoding = "bgr8";
          bgr_image->step = static_cast<uint32_t>(bgr.step);
          bgr_image->data.assign(bgr.datastart, bgr.dataend);
          image_pub_->publish(std::move(bgr_image));
        }
      }
      if (info_pub_->is_activated()) info_pub_->publish(info);
    }

    // Defined cycle — see this function's opening comment. Runs whether
    // this iteration succeeded or failed, so it caps the loop's rate
    // either way rather than only backing off on failure.
    const auto elapsed = std::chrono::steady_clock::now() - cycle_start;
    const auto remaining =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(target_period) - elapsed;
    if (remaining > std::chrono::steady_clock::duration::zero()) {
      std::this_thread::sleep_for(remaining);
    }
  }
}

}  // namespace mserve_camera

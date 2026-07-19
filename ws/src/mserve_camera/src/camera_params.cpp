#include "mserve_camera/camera_node.hpp"
#include "mserve_camera/camera_limits.hpp"

#include <lifecycle_msgs/msg/state.hpp>

#include "mserve_utils/utils.hpp"

namespace mserve_camera {

void CameraNode::declare_params()
{
  this->declare_parameter<std::string>("device", "/dev/video0");

  this->declare_parameter<int64_t>("width", 640,
    mserve_utils::make_int_range_descriptor("Capture width (px)", kWidthMin, kWidthMax));
  this->declare_parameter<int64_t>("height", 480,
    mserve_utils::make_int_range_descriptor("Capture height (px)", kHeightMin, kHeightMax));

  // REP-103 optical frame — matches mserve_camera.xacro/mserve_depth_camera.xacro's
  // camera_link -> camera_link_optical joint, not the physical mount frame.
  this->declare_parameter<std::string>("frame_id", "camera_link_optical");

  this->declare_parameter<int64_t>("jpeg_quality", kJpegQualityDefault,
    mserve_utils::make_int_range_descriptor(
      "JPEG quality for camera/image_raw/compressed", kJpegQualityMin, kJpegQualityMax));

  // Physical mount is upside down on this chassis — rotate 180 in software
  // rather than relying on a mount fix. Applies to both camera/image_raw
  // (now published as bgr8, not raw YUYV — see convert_and_flip()) and
  // camera/image_raw/compressed.
  this->declare_parameter<bool>("flip_180", true);
}

void CameraNode::load_params() {
  device_       = get_parameter("device").as_string();
  width_        = static_cast<int>(get_parameter("width").as_int());
  height_       = static_cast<int>(get_parameter("height").as_int());
  frame_id_     = get_parameter("frame_id").as_string();
  jpeg_quality_ = static_cast<int>(get_parameter("jpeg_quality").as_int());
  flip_180_     = get_parameter("flip_180").as_bool();

  // Uncalibrated placeholder CameraInfo — width/height/frame_id are real,
  // K/D/R/P are left zeroed until an actual calibration is run. Any consumer
  // that needs real intrinsics should re-run camera_calibration first.
  camera_info_ = sensor_msgs::msg::CameraInfo();
  camera_info_.width  = static_cast<uint32_t>(width_);
  camera_info_.height = static_cast<uint32_t>(height_);
  camera_info_.header.frame_id = frame_id_;
}

rcl_interfaces::msg::SetParametersResult CameraNode::on_parameters(const std::vector<rclcpp::Parameter> & params){
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  const bool is_unconfigured =
    get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;

  for (const auto & p : params) {
    const auto & name = p.get_name();

    // Device/format params require reopening the V4L2 device — only safe in UNCONFIGURED.
    const bool is_hw_param = name == "device" || name == "width" || name == "height";

    if (is_hw_param && !is_unconfigured) {
      result.successful = false;
      result.reason = name + " can only be changed in UNCONFIGURED state";
      return result;
    }
  }
  return result;
}

}  // namespace mserve_camera

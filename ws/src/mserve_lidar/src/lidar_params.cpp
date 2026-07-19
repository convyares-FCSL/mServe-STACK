#include "mserve_lidar/lidar_node.hpp"
#include "mserve_lidar/lidar_limits.hpp"

#include <lifecycle_msgs/msg/state.hpp>

#include "mserve_utils/utils.hpp"

namespace mserve_lidar {

void LidarNode::declare_params()
{
  this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
  this->declare_parameter<int64_t>("baudrate", kDefaultBaudrate);

  // Matches mserve_lidar.xacro's lidar_link — not an optical frame like the
  // camera's, since LaserScan has no REP-103 optical-frame convention.
  this->declare_parameter<std::string>("frame_id", "lidar_link");

  // Empty = let the device pick its own typical/default scan mode. Only
  // needs a value if you want to force e.g. a boosted/sensitivity mode name
  // reported by `getAllSupportedScanModes()` for your specific unit.
  this->declare_parameter<std::string>("scan_mode", "");

  this->declare_parameter<bool>("inverted", false);

  this->declare_parameter<double>("scan_rate_hz", 15.0,
    mserve_utils::make_double_range_descriptor(
      "Ceiling on capture_loop()'s iteration rate — see lidar_node.hpp",
      kScanRateHzMin, kScanRateHzMax));
}

void LidarNode::load_params() {
  device_       = get_parameter("device").as_string();
  baudrate_     = static_cast<int>(get_parameter("baudrate").as_int());
  frame_id_     = get_parameter("frame_id").as_string();
  scan_mode_    = get_parameter("scan_mode").as_string();
  inverted_     = get_parameter("inverted").as_bool();
  scan_rate_hz_ = get_parameter("scan_rate_hz").as_double();
}

rcl_interfaces::msg::SetParametersResult LidarNode::on_parameters(const std::vector<rclcpp::Parameter> & params){
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  const bool is_unconfigured =
    get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;

  for (const auto & p : params) {
    const auto & name = p.get_name();

    // device/baudrate require reconnecting to the lidar — only safe in UNCONFIGURED.
    const bool is_hw_param = name == "device" || name == "baudrate";

    if (is_hw_param && !is_unconfigured) {
      result.successful = false;
      result.reason = name + " can only be changed in UNCONFIGURED state";
      return result;
    }
  }
  return result;
}

}  // namespace mserve_lidar

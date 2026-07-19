#include "mserve_lidar/lidar_node.hpp"
#include "mserve_lidar/lidar_limits.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <limits>
#include <thread>

namespace mserve_lidar {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double deg2rad(double deg) { return deg * kPi / 180.0; }

// angle_z_q14 is a 14-bit fixed-point fraction of the SDK's 90-degree unit —
// see the SDK's own getAngle() helper in rplidar_node.cpp for the reference
// formula this mirrors.
float node_angle_deg(const sl_lidar_response_measurement_node_hq_t & node) {
  return node.angle_z_q14 * 90.f / 16384.f;
}

}  // namespace

// ==============================================================================
// Construction / destruction
// ==============================================================================

LidarNode::LidarNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("mserve_lidar", options)
{
  declare_params();
  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) { return on_parameters(p); });
}

LidarNode::~LidarNode() {
  capturing_ = false;
  if (capture_thread_.joinable()) capture_thread_.join();
  disconnect();
}

// ==============================================================================
// Lifecycle
// ==============================================================================

LidarNode::CallbackReturn LidarNode::on_configure(const rclcpp_lifecycle::State &) {
  load_params();

  if (!connect()) {
    disconnect();
    return CallbackReturn::FAILURE;
  }

  if (!check_device_info() || !check_health()) {
    disconnect();
    return CallbackReturn::FAILURE;
  }

  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::KeepLast(10)));

  RCLCPP_INFO(get_logger(), "Configured — %s @ %d baud, frame_id=%s",
    device_.c_str(), baudrate_, frame_id_.c_str());
  return CallbackReturn::SUCCESS;
}

LidarNode::CallbackReturn LidarNode::on_activate(const rclcpp_lifecycle::State &) {
  driver_->setMotorSpeed();  // DEFAULT_MOTOR_SPEED sentinel — device picks its own default RPM

  if (!set_scan_mode()) {
    driver_->stop();
    driver_->setMotorSpeed(0);
    return CallbackReturn::FAILURE;
  }

  scan_pub_->on_activate();

  capturing_ = true;
  capture_thread_ = std::thread(&LidarNode::capture_loop, this);

  RCLCPP_INFO(get_logger(), "Activated — publishing scan (range_max=%.1fm)", max_distance_m_);
  return CallbackReturn::SUCCESS;
}

LidarNode::CallbackReturn LidarNode::on_deactivate(const rclcpp_lifecycle::State &) {
  capturing_ = false;
  if (capture_thread_.joinable()) capture_thread_.join();

  if (driver_) { driver_->stop(); driver_->setMotorSpeed(0); }

  scan_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

LidarNode::CallbackReturn LidarNode::on_cleanup(const rclcpp_lifecycle::State &) {
  capturing_ = false;
  if (capture_thread_.joinable()) capture_thread_.join();
  disconnect();
  scan_pub_.reset();
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

LidarNode::CallbackReturn LidarNode::on_shutdown(const rclcpp_lifecycle::State &) {
  capturing_ = false;
  if (capture_thread_.joinable()) capture_thread_.join();
  if (driver_) { driver_->stop(); driver_->setMotorSpeed(0); }
  disconnect();
  RCLCPP_INFO(get_logger(), "Shutdown");
  return CallbackReturn::SUCCESS;
}

// ==============================================================================
// SDK connection helpers
// ==============================================================================

bool LidarNode::connect() {
  auto drv_result = sl::createLidarDriver();
  if (!drv_result) {
    RCLCPP_ERROR(get_logger(), "Failed to construct RPLIDAR driver");
    return false;
  }
  driver_ = *drv_result;

  auto channel_result = sl::createSerialPortChannel(device_, baudrate_);
  if (!channel_result) {
    RCLCPP_ERROR(get_logger(), "Failed to construct serial channel for %s", device_.c_str());
    return false;
  }
  channel_ = *channel_result;

  if (SL_IS_FAIL(driver_->connect(channel_))) {
    RCLCPP_ERROR(get_logger(), "Failed to connect to RPLIDAR on %s @ %d baud",
      device_.c_str(), baudrate_);
    return false;
  }
  return true;
}

bool LidarNode::check_device_info() {
  sl_lidar_response_device_info_t info;
  if (SL_IS_FAIL(driver_->getDeviceInfo(info))) {
    RCLCPP_ERROR(get_logger(), "Failed to read RPLIDAR device info");
    return false;
  }

  char serial[33] = {0};
  for (int i = 0; i < 16; ++i) { std::snprintf(serial + i * 2, 3, "%02X", info.serialnum[i]); }
  RCLCPP_INFO(get_logger(), "RPLIDAR S/N %s, firmware %d.%02d, hardware rev %d",
    serial, info.firmware_version >> 8, info.firmware_version & 0xFF, info.hardware_version);
  return true;
}

bool LidarNode::check_health() {
  sl_lidar_response_device_health_t health;
  if (SL_IS_FAIL(driver_->getHealth(health))) {
    RCLCPP_ERROR(get_logger(), "Failed to read RPLIDAR health status");
    return false;
  }
  if (health.status == SL_LIDAR_STATUS_ERROR) {
    RCLCPP_ERROR(get_logger(), "RPLIDAR reports an internal error — power-cycle the device");
    return false;
  }
  if (health.status == SL_LIDAR_STATUS_WARNING) {
    RCLCPP_WARN(get_logger(), "RPLIDAR health status: warning");
  }
  return true;
}

bool LidarNode::set_scan_mode() {
  sl::LidarScanMode used_mode;
  sl_result result;

  if (scan_mode_.empty()) {
    result = driver_->startScan(false /* not forced */, true /* typical scan mode */, 0, &used_mode);
  } else {
    std::vector<sl::LidarScanMode> supported;
    result = driver_->getAllSupportedScanModes(supported);
    if (SL_IS_FAIL(result)) {
      RCLCPP_ERROR(get_logger(), "Failed to query supported scan modes");
      return false;
    }

    const auto it = std::find_if(supported.begin(), supported.end(),
      [this](const sl::LidarScanMode & m) { return scan_mode_ == m.scan_mode; });
    if (it == supported.end()) {
      RCLCPP_ERROR(get_logger(), "scan_mode '%s' not supported by this device", scan_mode_.c_str());
      return false;
    }
    result = driver_->startScanExpress(false, it->id, 0, &used_mode);
  }

  if (SL_IS_FAIL(result)) {
    RCLCPP_ERROR(get_logger(), "Failed to start scan: %08x", result);
    return false;
  }

  max_distance_m_ = std::min(used_mode.max_distance, kMaxRangeM);
  RCLCPP_INFO(get_logger(), "Scan mode: %s, mode max_distance=%.1fm, published range_max=%.1fm",
    used_mode.scan_mode, used_mode.max_distance, max_distance_m_);
  return true;
}

void LidarNode::disconnect() {
  if (driver_) { driver_->disconnect(); delete driver_; driver_ = nullptr; }
  // The SDK docs require the channel to outlive the driver's use of it, and
  // ILidarDriver doesn't take ownership — deleting it is on us.
  if (channel_) { delete channel_; channel_ = nullptr; }
}

// ==============================================================================
// Capture loop
// ==============================================================================

void LidarNode::capture_loop() {
  // Paced to a defined cycle rather than trusting grabScanDataHq() to
  // naturally rate-limit this loop by blocking — same reasoning as
  // mserve_camera's capture_loop(), which needed this after a real
  // hardware incident (2026-07-19): a loop around a supposedly-blocking
  // hardware call still needs its own independent ceiling, since nothing
  // guarantees the call keeps blocking normally under a device failure.
  const auto target_period = std::chrono::duration<double>(1.0 / scan_rate_hz_);
  std::vector<sl_lidar_response_measurement_node_hq_t> nodes(8192);
  int consecutive_failures = 0;
  auto last_health_log = std::chrono::steady_clock::now();

  while (capturing_.load()) {
    const auto cycle_start = std::chrono::steady_clock::now();
    size_t count = nodes.size();
    const auto start = now();
    const sl_result result = driver_->grabScanDataHq(nodes.data(), count);
    const double scan_duration = (now() - start).seconds();

    if (SL_IS_FAIL(result)) {
      ++consecutive_failures;
      if (cycle_start - last_health_log >= std::chrono::seconds(5)) {
        RCLCPP_WARN(
          get_logger(), "capture_loop: %d consecutive failures, device may be gone",
          consecutive_failures);
        last_health_log = cycle_start;
      }
    } else {
      consecutive_failures = 0;
      driver_->ascendScanData(nodes.data(), count);
      if (count >= 2) {  // need at least 2 points for angle_increment
        publish_scan(nodes, count, start, scan_duration);
      }
    }

    const auto elapsed = std::chrono::steady_clock::now() - cycle_start;
    const auto remaining =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(target_period) - elapsed;
    if (remaining > std::chrono::steady_clock::duration::zero()) {
      std::this_thread::sleep_for(remaining);
    }
  }
}

void LidarNode::publish_scan(
  const std::vector<sl_lidar_response_measurement_node_hq_t> & nodes,
  size_t count, rclcpp::Time stamp, double scan_duration)
{
  // Deliberately NOT trimming leading/trailing no-return (dist_mm_q2 == 0)
  // samples here — this used to shrink the published array to only the
  // "real data" span, but that means angle_min/angle_max/array-size varied
  // scan-to-scan (however many no-return samples happened to land at the
  // edges that revolution). SLAM/Nav2 consumers (slam_toolbox's Karto
  // backend confirmed, and Nav2's costmap raytracing works the same way)
  // register a laser's geometry from the first scan they see and expect
  // every later scan to match it exactly — a varying array size meant
  // slam_toolbox silently rejected nearly every scan after the first
  // ("LaserRangeScan contains N range readings, expected M"), so /map never
  // updated. Fixed by always publishing the full raw buffer (indices
  // 0..count-1) and representing no-return samples as `range = infinity`
  // wherever they land, which is what the standard LaserScan convention
  // (and the mid-scan case just below) already does — no data is lost,
  // just no longer silently dropped from the array's edges.
  const size_t node_count = count;

  auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan->header.stamp = stamp;
  scan->header.frame_id = frame_id_;

  // SDK angles increase clockwise looking down at the device; negate to
  // follow REP-103 (counter-clockwise-positive about +Z, X-forward).
  const float angle_min_deg = node_angle_deg(nodes[0]);
  const float angle_max_deg = node_angle_deg(nodes[node_count - 1]);
  scan->angle_min = deg2rad(-angle_max_deg);
  scan->angle_max = deg2rad(-angle_min_deg);
  scan->angle_increment = (scan->angle_max - scan->angle_min) / static_cast<double>(node_count - 1);

  scan->scan_time = scan_duration;
  scan->time_increment = scan_duration / static_cast<double>(node_count);
  scan->range_min = kMinRangeM;
  scan->range_max = max_distance_m_;

  scan->ranges.resize(node_count);
  scan->intensities.resize(node_count);
  for (size_t i = 0; i < node_count; ++i) {
    // Reversed index — we negated the angle direction above, so ranges[0]
    // must correspond to angle_min, i.e. the last (highest-angle) SDK sample.
    const auto & node = nodes[node_count - 1 - i];
    const float range_m = static_cast<float>(node.dist_mm_q2) / 4000.0f;
    scan->ranges[i] = (range_m == 0.0f) ? std::numeric_limits<float>::infinity() : range_m;
    scan->intensities[i] = static_cast<float>(node.quality >> 2);
  }

  if (inverted_) {
    std::reverse(scan->ranges.begin(), scan->ranges.end());
    std::reverse(scan->intensities.begin(), scan->intensities.end());
  }

  if (scan_pub_->is_activated()) scan_pub_->publish(std::move(scan));
}

}  // namespace mserve_lidar

#pragma once

namespace mserve_lidar {

// RPLIDAR C1 serial link runs at a fixed 460800 baud — no other rate is
// valid, so there's nothing to bound here. Kept as a named constant (not a
// magic number in lidar_params.cpp) for the same reason mserve_camera keeps
// kWidthMin/kWidthMax in its own limits header.
constexpr int kDefaultBaudrate = 460800;

// sl_lidar_response_measurement_node_hq_t dist_mm_q2 is a 2-bit fixed-point
// mm value; 0 means "no return" (out of range / no reflection), not "0m".
constexpr float kMinRangeM = 0.05f;

// RPLIDAR C1 datasheet max range. The SDK's negotiated LidarScanMode also
// reports a max_distance (e.g. "DenseBoost" reports 40.0m) but that's a
// nominal figure for the scan mode's sample-rate/quality tradeoff, not a
// promise of usable returns out that far — clamp range_max to the real
// spec so LaserScan consumers (RViz, the web UI) don't scale views around
// a number the sensor can't actually deliver.
constexpr float kMaxRangeM = 12.0f;

}  // namespace mserve_lidar

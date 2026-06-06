#pragma once

#include <memory>
#include <mutex>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <mserve_interfaces/msg/compressor_telemetry.hpp>

namespace hyfleet_booster {

// Thread-safe telemetry cache. BoosterNode owns the subscription and writes here;
// BT condition nodes and tick_tree_once() read from here. One source of truth.
class BoosterTelemetryCache {
public:
    using Msg = mserve_interfaces::msg::CompressorTelemetry;

    void update(std::shared_ptr<const Msg> msg, rclcpp::Time stamp)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        msg_   = std::move(msg);
        stamp_ = stamp;
    }

    std::pair<std::shared_ptr<const Msg>, rclcpp::Time> latest() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return {msg_, stamp_};
    }

private:
    mutable std::mutex mutex_;
    std::shared_ptr<const Msg> msg_;
    rclcpp::Time stamp_{};
};

} // namespace hyfleet_booster

#pragma once
#include <memory>
#include <mutex>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <mserve_interfaces/msg/compressor_telemetry.hpp>

namespace hyfleet_compressor {

// Thread-safe telemetry cache. CompressorNode owns the subscription and writes here; One source of truth.
class CompressorTelemetryCache {
public:
    using Msg = mserve_interfaces::msg::CompressorTelemetry;

    void update(std::shared_ptr<const Msg> msg, rclcpp::Time stamp) { 
        std::lock_guard<std::mutex> lock(mutex_);
        msg_   = std::move(msg);
        stamp_ = stamp;
    }

    std::pair<std::shared_ptr<const Msg>, rclcpp::Time> latest() const { 
        std::lock_guard<std::mutex> lock(mutex_);
        return {msg_, stamp_};
    }

private:
    mutable std::mutex mutex_;
    std::shared_ptr<const Msg> msg_;
    rclcpp::Time stamp_{};
};

} // namespace hyfleet_compressor
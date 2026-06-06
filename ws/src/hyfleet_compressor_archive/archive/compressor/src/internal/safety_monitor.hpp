#pragma once

#include <string>

#include "internal/compressor_config.hpp"
#include "internal/device_types.hpp"
#include "internal/telemetry_subscriber.hpp"

namespace compressor
{

struct SafetyStatus
{
  bool inhibited = false;
  std::string message;
  DeviceCommands commands;
};

class SafetyMonitor
{
public:
  explicit SafetyMonitor(SafetyConfig config);

  SafetyStatus evaluate(const CompressorTelemetryUpdate & telemetry) const;
  SafetyStatus telemetry_missing() const;

private:
  SafetyConfig config_;
};

}  // namespace compressor

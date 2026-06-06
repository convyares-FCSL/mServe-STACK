#pragma once

#include "internal/booster.hpp"
#include "internal/compressor_types.hpp"
#include "internal/device_types.hpp"

namespace compressor
{

class BoosterManager
{
public:
  BoosterManager(BoosterConfig low_booster_config, BoosterConfig high_booster_config);

  void apply_control(CompressorControl request);
  void set_telemetry(
    const BoosterTelemetry & low_booster_telemetry,
    const BoosterTelemetry & high_booster_telemetry);
  DeviceCommands update();
  CompressorStatus status() const;

  void reset();

private:
  Booster low_booster_;
  Booster high_booster_;
};

}  // namespace compressor

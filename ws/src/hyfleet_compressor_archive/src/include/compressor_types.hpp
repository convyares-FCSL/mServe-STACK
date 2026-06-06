#pragma once

#include <cstdint>
#include <string>

namespace hyfleet_compressor
{

enum class CompressorCommand : uint8_t
{
  START = 1,
  STOP = 2,
  SAFE_STOP = 3
};

enum class CompressorTarget : uint8_t
{
  LOW_BOOSTER = 1,
  HIGH_BOOSTER = 2,
  SYNC_BOOSTERS = 3
};


struct CompressorControl
{
  CompressorCommand command;
  CompressorTarget target;
  double target_pressure = 0.0;
};

struct BoosterStatus
{
  double pressure = 0.0;
  bool target_reached = false;
};

struct CompressorStatus
{
  BoosterStatus low_booster;
  BoosterStatus high_booster;
};

}  // namespace hyfleet_compressor

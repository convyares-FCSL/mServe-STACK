#pragma once

#include <cstdint>
#include <string>

namespace compressor
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

enum class CompressionPhaseState : uint8_t
{
  WAIT_COMMAND = 0,
  RAMP_VFD_UP = 1,
  VFD_DELAY = 2,
  WAIT_RAMP = 3,
  WAIT_STABILIZATION = 4,
  ENERGISE_VALVE = 5,
  PCSV_ON = 6,
  PCSV_OFF = 7,
  DEENERGISE_VALVE = 8,
  RAMP_VFD_DOWN = 9,
  WAIT_STOP = 10
};

struct CompressorControl
{
  CompressorCommand command;
  CompressorTarget target;
  double target_pressure = 0.0;
};

struct BoosterTelemetry
{
  double input_pressure = 0.0;
  double output_pressure = 0.0;
  double compression_ratio = 0.0;
  double input_temperature = 0.0;
  double output_temperature = 0.0;
  double vfd_energy = 0.0;
  double vfd_speed = 0.0;
};

struct BoosterConfig
{
  std::string name;
  std::string inlet_sv_id;
  std::string hpu_sv_id;
  std::string vfd_id;
  std::string pcsv_id;
  double off_inlet_pressure = 0.2;
  double target_deadband = 0.5;
  double vfd_target_speed = 1000.0;
  double vfd_ramp_tolerance = 25.0;
  double vfd_stop_threshold = 25.0;
  double stability_tolerance = 0.05;
  double pcsv_cpm = 10.0;
  uint32_t stabilization_sample_count = 3;
  uint32_t vfd_delay_cycles = 1;
  uint32_t wait_stabilization_cycles = 3;
};

struct BoosterStatus
{
  std::string name;
  bool active = false;
  bool target_reached = false;
  CompressionPhaseState phase = CompressionPhaseState::WAIT_COMMAND;
  std::string message;
  BoosterTelemetry telemetry;
  double target_pressure = 0.0;
  double pressure = 0.0;
  int cpm = 0;
  double hours_run = 0.0;
};

struct CompressorStatus
{
  BoosterStatus low_booster;
  BoosterStatus high_booster;
};

}  // namespace compressor

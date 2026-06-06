#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "internal/compressor_types.hpp"
#include "internal/device_types.hpp"

namespace compressor
{

class Booster
{
public:
  explicit Booster(BoosterConfig config = {});

  void request_start(double target_pressure);
  void request_stop();
  void request_safe_stop();
  void reset();
  DeviceCommands update();

  void set_telemetry(const BoosterTelemetry & telemetry);

  BoosterStatus status() const;

private:
  void update_status(std::string message, bool target_reached = false);
  void reset_inlet_samples();
  void append_inlet_sample(double pressure);
  bool is_inlet_stable() const;
  void advance_timers();
  void update_hours_run();

  struct CycleTimer
  {
    void start();
    void stop();
    void update();
    bool has_elapsed(uint32_t timeout_cycles) const;

    uint32_t elapsed = 0;
    bool active = false;
  };

  BoosterConfig config_;
  BoosterStatus status_;
  BoosterTelemetry telemetry_;
  bool start_requested_ = false;
  bool safe_stop_requested_ = false;
  CompressionPhaseState phase_ = CompressionPhaseState::WAIT_COMMAND;
  std::vector<double> inlet_sample_buffer_;
  uint32_t inlet_sample_index_ = 0;
  uint32_t samples_collected_ = 0;
  CycleTimer vfd_delay_;
  CycleTimer wait_stabilization_;
};

}  // namespace compressor

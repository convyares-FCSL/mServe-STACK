#include "internal/booster.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace compressor
{

Booster::Booster(BoosterConfig config)
: config_(std::move(config))
{
  reset();
}

void Booster::request_start(double target_pressure)
{
  start_requested_ = true;
  safe_stop_requested_ = false;
  status_.target_reached = false;
  status_.target_pressure = target_pressure;
}

void Booster::request_stop()
{
  start_requested_ = false;
  safe_stop_requested_ = false;
  status_.target_reached = false;
}

void Booster::request_safe_stop()
{
  start_requested_ = false;
  safe_stop_requested_ = true;
  status_.target_reached = false;
}

void Booster::reset()
{
  status_ = {};
  telemetry_ = {};
  start_requested_ = false;
  safe_stop_requested_ = false;
  phase_ = CompressionPhaseState::WAIT_COMMAND;
  reset_inlet_samples();
  vfd_delay_.stop();
  wait_stabilization_.stop();
  update_status(config_.name + ": Waiting initiation");
}

DeviceCommands Booster::update()
{
  DeviceCommands commands;
  auto finish = [this, &commands](std::string message, bool target_reached = false) {
      update_status(std::move(message), target_reached);
      return commands;
    };

  advance_timers();
  update_hours_run();

  const double inlet_pressure = telemetry_.input_pressure;
  const double outlet_pressure = telemetry_.output_pressure;

  switch (phase_) {
    case CompressionPhaseState::WAIT_COMMAND:
      safe_stop_requested_ = false;
      if (start_requested_) {
        if (outlet_pressure >= status_.target_pressure - config_.target_deadband) {
          start_requested_ = false;
          return finish(config_.name + ": Target pressure reached, holding", true);
        }
        commands.sv.push_back({config_.inlet_sv_id, true});
        reset_inlet_samples();
        phase_ = CompressionPhaseState::RAMP_VFD_UP;
      }
      return finish(config_.name + ": Waiting initiation");

    case CompressionPhaseState::RAMP_VFD_UP:
      if (safe_stop_requested_) {
        commands.sv.push_back({config_.inlet_sv_id, false});
        phase_ = CompressionPhaseState::WAIT_COMMAND;
        return finish(config_.name + ": Safe stop - closing inlet");
      }
      if (!start_requested_) {
        commands.sv.push_back({config_.inlet_sv_id, false});
        phase_ = CompressionPhaseState::WAIT_COMMAND;
        return finish(config_.name + ": Stop requested - closing inlet");
      }
      append_inlet_sample(inlet_pressure);
      if (is_inlet_stable()) {
        commands.vfd.push_back({config_.vfd_id, true, true, config_.vfd_target_speed});
        vfd_delay_.start();
        phase_ = CompressionPhaseState::VFD_DELAY;
      }
      return finish(config_.name + ": Waiting inlet pressure stabilise");

    case CompressionPhaseState::VFD_DELAY:
      if (safe_stop_requested_) {
        vfd_delay_.stop();
        phase_ = CompressionPhaseState::RAMP_VFD_DOWN;
        return finish(config_.name + ": Safe stop - shutting down");
      }
      if (!start_requested_) {
        vfd_delay_.stop();
        phase_ = CompressionPhaseState::RAMP_VFD_DOWN;
        return finish(config_.name + ": Stop requested - shutting down");
      }
      if (vfd_delay_.has_elapsed(config_.vfd_delay_cycles)) {
        vfd_delay_.stop();
        phase_ = CompressionPhaseState::WAIT_RAMP;
      }
      return finish(config_.name + ": VFD delay");

    case CompressionPhaseState::WAIT_RAMP:
      if (safe_stop_requested_) {
        phase_ = CompressionPhaseState::RAMP_VFD_DOWN;
        return finish(config_.name + ": Safe stop - shutting down");
      }
      if (!start_requested_) {
        phase_ = CompressionPhaseState::RAMP_VFD_DOWN;
        return finish(config_.name + ": Stop requested - shutting down");
      }
      if (telemetry_.vfd_speed >= config_.vfd_target_speed - config_.vfd_ramp_tolerance) {
        wait_stabilization_.start();
        phase_ = CompressionPhaseState::WAIT_STABILIZATION;
      }
      return finish(config_.name + ": Waiting for ramp completion");

    case CompressionPhaseState::WAIT_STABILIZATION:
      if (safe_stop_requested_) {
        wait_stabilization_.stop();
        phase_ = CompressionPhaseState::RAMP_VFD_DOWN;
        return finish(config_.name + ": Safe stop - shutting down");
      }
      if (!start_requested_) {
        wait_stabilization_.stop();
        phase_ = CompressionPhaseState::RAMP_VFD_DOWN;
        return finish(config_.name + ": Stop requested - shutting down");
      }
      if (wait_stabilization_.has_elapsed(config_.wait_stabilization_cycles)) {
        wait_stabilization_.stop();
        phase_ = CompressionPhaseState::ENERGISE_VALVE;
      }
      return finish(config_.name + ": Waiting for stabilization");

    case CompressionPhaseState::ENERGISE_VALVE:
      if (safe_stop_requested_) {
        phase_ = CompressionPhaseState::RAMP_VFD_DOWN;
        return finish(config_.name + ": Safe stop - shutting down");
      }
      if (!start_requested_) {
        phase_ = CompressionPhaseState::RAMP_VFD_DOWN;
        return finish(config_.name + ": Stop requested - shutting down");
      }
      commands.sv.push_back({config_.hpu_sv_id, true});
      phase_ = CompressionPhaseState::PCSV_ON;
      return finish(config_.name + ": Energising HPU solenoid");

    case CompressionPhaseState::PCSV_ON:
      if (safe_stop_requested_ || !start_requested_) {
        commands.pcsv.push_back({config_.pcsv_id, false, 0.0});
        status_.cpm = 0;
        phase_ = CompressionPhaseState::DEENERGISE_VALVE;
        return finish(config_.name + ": Stop requested - ceasing compression");
      }
      if (inlet_pressure < config_.off_inlet_pressure) {
        commands.pcsv.push_back({config_.pcsv_id, false, 0.0});
        status_.cpm = 0;
        phase_ = CompressionPhaseState::DEENERGISE_VALVE;
        return finish(config_.name + ": Inlet pressure too low");
      }
      if (outlet_pressure >= status_.target_pressure) {
        commands.pcsv.push_back({config_.pcsv_id, false, 0.0});
        start_requested_ = false;
        phase_ = CompressionPhaseState::DEENERGISE_VALVE;
        return finish(config_.name + ": Target pressure reached, commencing shutdown", true);
      }
      commands.pcsv.push_back({config_.pcsv_id, true, config_.pcsv_cpm});
      status_.cpm = static_cast<int>(std::lround(config_.pcsv_cpm));
      return finish(config_.name + ": PCSV active, system running");

    case CompressionPhaseState::PCSV_OFF:
      commands.pcsv.push_back({config_.pcsv_id, false, 0.0});
      status_.cpm = 0;
      phase_ = CompressionPhaseState::DEENERGISE_VALVE;
      return finish(config_.name + ": Stopping PCSV");

    case CompressionPhaseState::DEENERGISE_VALVE:
      commands.sv.push_back({config_.hpu_sv_id, false});
      phase_ = CompressionPhaseState::RAMP_VFD_DOWN;
      return finish(config_.name + ": De-energising HPU solenoid");

    case CompressionPhaseState::RAMP_VFD_DOWN:
      commands.vfd.push_back({config_.vfd_id, true, false, 0.0});
      phase_ = CompressionPhaseState::WAIT_STOP;
      return finish(config_.name + ": Ramping VFD down");

    case CompressionPhaseState::WAIT_STOP:
      if (telemetry_.vfd_speed <= config_.vfd_stop_threshold) {
        commands.sv.push_back({config_.inlet_sv_id, false});
        phase_ = CompressionPhaseState::WAIT_COMMAND;
        safe_stop_requested_ = false;
        return finish(config_.name + ": Waiting ramp down completion");
      }
      return finish(config_.name + ": Waiting ramp down completion");
  }

  return finish(config_.name + ": Unknown state");
}

void Booster::set_telemetry(const BoosterTelemetry & telemetry)
{
  telemetry_ = telemetry;
  status_.telemetry = telemetry_;
  status_.pressure = telemetry_.output_pressure;
}

BoosterStatus Booster::status() const
{
  return status_;
}

void Booster::update_status(std::string message, bool target_reached)
{
  status_.name = config_.name;
  status_.phase = phase_;
  status_.message = std::move(message);
  if (target_reached) {
    status_.target_reached = true;
  }
  status_.active = phase_ != CompressionPhaseState::WAIT_COMMAND;
  status_.telemetry = telemetry_;
  status_.pressure = telemetry_.output_pressure;
  if (phase_ != CompressionPhaseState::PCSV_ON) {
    status_.cpm = 0;
  }
}

void Booster::reset_inlet_samples()
{
  inlet_sample_buffer_.assign(
    std::max<uint32_t>(config_.stabilization_sample_count, 1),
    0.0);
  inlet_sample_index_ = 0;
  samples_collected_ = 0;
}

void Booster::append_inlet_sample(double pressure)
{
  inlet_sample_buffer_[inlet_sample_index_] = pressure;
  inlet_sample_index_ = (inlet_sample_index_ + 1) % inlet_sample_buffer_.size();
  if (samples_collected_ < inlet_sample_buffer_.size()) {
    ++samples_collected_;
  }
}

bool Booster::is_inlet_stable() const
{
  if (samples_collected_ < config_.stabilization_sample_count) {
    return false;
  }

  const uint32_t sample_count =
    std::min<uint32_t>(config_.stabilization_sample_count, inlet_sample_buffer_.size());
  const uint32_t buffer_size = inlet_sample_buffer_.size();
  double average = 0.0;

  for (uint32_t i = 0; i < sample_count; ++i) {
    const uint32_t index = (inlet_sample_index_ + buffer_size - sample_count + i) % buffer_size;
    average += inlet_sample_buffer_[index];
  }
  average /= static_cast<double>(sample_count);

  for (uint32_t i = 0; i < sample_count; ++i) {
    const uint32_t index = (inlet_sample_index_ + buffer_size - sample_count + i) % buffer_size;
    if (std::abs(inlet_sample_buffer_[index] - average) > config_.stability_tolerance) {
      return false;
    }
  }
  return true;
}

void Booster::advance_timers()
{
  vfd_delay_.update();
  wait_stabilization_.update();
}

void Booster::CycleTimer::start()
{
  elapsed = 0;
  active = true;
}

void Booster::CycleTimer::stop()
{
  elapsed = 0;
  active = false;
}

void Booster::CycleTimer::update()
{
  if (active) {
    ++elapsed;
  }
}

bool Booster::CycleTimer::has_elapsed(uint32_t timeout_cycles) const
{
  return active && elapsed >= timeout_cycles;
}

void Booster::update_hours_run()
{
  if (phase_ == CompressionPhaseState::PCSV_ON) {
    status_.hours_run += 1.0 / 3600.0;
  }
}

}  // namespace compressor

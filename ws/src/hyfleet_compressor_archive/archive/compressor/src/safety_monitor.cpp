#include "internal/safety_monitor.hpp"

#include <cstdint>
#include <utility>
#include <vector>

namespace compressor
{

namespace
{
void set_heaters(DeviceCommands & commands, const std::vector<uint8_t> & heater_ids, bool state)
{
  for (const auto heater_id : heater_ids) {
    commands.heater.push_back({heater_id, state});
  }
}
}  // namespace

SafetyMonitor::SafetyMonitor(SafetyConfig config)
: config_(std::move(config))
{
}

SafetyStatus SafetyMonitor::evaluate(const CompressorTelemetryUpdate & telemetry) const
{
  SafetyStatus status;

  if (telemetry.hpu_temperature > config_.oil_temp_inhibit_high_celsius) {
    status.inhibited = true;
    status.message = "Oil temperature too high, inhibiting compressor operation";
    set_heaters(status.commands, config_.heater_ids, false);
    return status;
  }

  if (telemetry.hpu_temperature < config_.oil_temp_inhibit_low_celsius) {
    status.inhibited = true;
    status.message = "Oil temperature too low, inhibiting compressor operation";
    set_heaters(status.commands, config_.heater_ids, true);
    return status;
  }

  for (const auto index : config_.inlet_temp_indices) {
    if (telemetry.hbu_temperatures[index] > config_.hbu_inlet_temp_limit_celsius) {
      status.inhibited = true;
      status.message = "Booster inlet temperature too high, inhibiting compressor operation";
      set_heaters(status.commands, config_.heater_ids, false);
      return status;
    }
  }

  for (const auto index : config_.outlet_temp_indices) {
    if (telemetry.hbu_temperatures[index] > config_.hbu_outlet_temp_limit_celsius) {
      status.inhibited = true;
      status.message = "Booster outlet temperature too high, inhibiting compressor operation";
      set_heaters(status.commands, config_.heater_ids, false);
      return status;
    }
  }

  for (const auto index : config_.chiller_temp_indices) {
    if (telemetry.hbu_temperatures[index] > config_.chiller_outlet_temp_limit_celsius) {
      status.inhibited = true;
      status.message = "Chiller outlet temperature too high, inhibiting compressor operation";
      set_heaters(status.commands, config_.heater_ids, false);
      return status;
    }
  }

  set_heaters(status.commands, config_.heater_ids, false);
  return status;
}

SafetyStatus SafetyMonitor::telemetry_missing() const
{
  SafetyStatus status;
  status.inhibited = true;
  status.message = "Compressor telemetry missing, inhibiting compressor operation";
  return status;
}

}  // namespace compressor

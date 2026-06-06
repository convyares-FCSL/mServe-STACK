#include "internal/booster_manager.hpp"

#include <utility>

namespace compressor
{

namespace
{
bool targets_low_booster(CompressorTarget target)
{
  return target == CompressorTarget::LOW_BOOSTER ||
         target == CompressorTarget::SYNC_BOOSTERS;
}

bool targets_high_booster(CompressorTarget target)
{
  return target == CompressorTarget::HIGH_BOOSTER ||
         target == CompressorTarget::SYNC_BOOSTERS;
}
}  // namespace

BoosterManager::BoosterManager(
  BoosterConfig low_booster_config,
  BoosterConfig high_booster_config)
: low_booster_(std::move(low_booster_config)),
  high_booster_(std::move(high_booster_config))
{
}

void BoosterManager::apply_control(CompressorControl request)
{
  switch (request.command) {
    case CompressorCommand::START:
      if (targets_low_booster(request.target)) {
        low_booster_.request_start(request.target_pressure);
      }
      if (targets_high_booster(request.target)) {
        high_booster_.request_start(request.target_pressure);
      }
      break;

    case CompressorCommand::STOP:
      if (targets_low_booster(request.target)) {
        low_booster_.request_stop();
      }
      if (targets_high_booster(request.target)) {
        high_booster_.request_stop();
      }
      break;

    case CompressorCommand::SAFE_STOP:
      if (targets_low_booster(request.target)) {
        low_booster_.request_safe_stop();
      }
      if (targets_high_booster(request.target)) {
        high_booster_.request_safe_stop();
      }
      break;
  }
}

void BoosterManager::set_telemetry(
  const BoosterTelemetry & low_booster_telemetry,
  const BoosterTelemetry & high_booster_telemetry)
{
  low_booster_.set_telemetry(low_booster_telemetry);
  high_booster_.set_telemetry(high_booster_telemetry);
}

DeviceCommands BoosterManager::update()
{
  DeviceCommands commands;
  append_device_commands(commands, low_booster_.update());
  append_device_commands(commands, high_booster_.update());
  return commands;
}

CompressorStatus BoosterManager::status() const
{
  return CompressorStatus{low_booster_.status(), high_booster_.status()};
}

void BoosterManager::reset()
{
  low_booster_.reset();
  high_booster_.reset();
}

}  // namespace compressor

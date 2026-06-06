#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace compressor
{

struct SV
{
  std::string device_id;
  bool state = false;
};

struct PCSV
{
  std::string device_id;
  bool enable = false;
  double cpm = 0.0;
};

struct VFD
{
  std::string device_id;
  bool enable = false;
  bool on_off = false;
  double speed_rpm = 0.0;
};

struct Heater
{
  uint8_t heater_id = 0;
  bool state = false;
};

struct DeviceCommands
{
  std::vector<SV> sv;
  std::vector<PCSV> pcsv;
  std::vector<VFD> vfd;
  std::vector<Heater> heater;
};

inline void append_device_commands(DeviceCommands & target, const DeviceCommands & source)
{
  target.sv.insert(target.sv.end(), source.sv.begin(), source.sv.end());
  target.pcsv.insert(target.pcsv.end(), source.pcsv.begin(), source.pcsv.end());
  target.vfd.insert(target.vfd.end(), source.vfd.begin(), source.vfd.end());
  target.heater.insert(target.heater.end(), source.heater.begin(), source.heater.end());
}

struct BoosterDeviceConfig
{
  std::string inlet_sv_id;
  std::string hpu_sv_id;
  std::string vfd_id;
  std::string pcsv_id;
};

struct DeviceServices
{
  std::string control_low_booster_service_name;
  std::string control_high_booster_service_name;
  std::string control_compressor_service_name;
};

struct DeviceLimits
{
  double pcsv_min_cpm = 0.0;
  double pcsv_max_cpm = 0.0;
  double vfd_min_speed_rpm = 0.0;
  double vfd_max_speed_rpm = 0.0;
};

struct DeviceCommandIds
{
  int32_t low_booster = 1;
  int32_t high_booster = 2;
  int32_t compressor = 1;
  std::vector<uint8_t> heater_ids{1, 2};
};

struct DeviceDispatcherConfig
{
  DeviceServices services;
  DeviceLimits limits;
  DeviceCommandIds command_ids;
  BoosterDeviceConfig low_booster;
  BoosterDeviceConfig high_booster;
  double command_ack_timeout_sec = 1.0;
};

}  // namespace compressor

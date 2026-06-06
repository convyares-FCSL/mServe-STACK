# Compressor process telemetry published by the PLC bridge at ~10 Hz

uint8 OFF = 0
uint8 STARTUP = 1
uint8 AUTO = 2
uint8 MANUAL = 3
uint8 LOCKOUT = 4

uint8 mode

# Devices 
float64[16] pt_bar              # Hydrogren Pressure Transducer
float64[12] tt_celsius          # Hydrogren + Coolant Temperature Transducer
bool[4] ps                      # Booster Position Sensor
float64[2] vfd_speed_rpm        # VFD speed
float64[2] vfd_energy_kj        # VFD Energy
float64[2] vfd_power_kw         # VFD Power
bool[2] heater_on               # Heater Status
float64 hpu_tt_celsius          # Hydrolic Tank Temperature
float64 hpu_ls_percent          # Hydrolic Tank Fill Percent

builtin_interfaces/Time timestamp

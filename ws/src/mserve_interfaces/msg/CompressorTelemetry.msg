# Compressor process telemetry published by the PLC bridge at ~10 Hz

uint8 OFF = 0
uint8 STARTUP = 1
uint8 AUTO = 2
uint8 MANUAL = 3
uint8 LOCKOUT = 4

uint8 mode

# Devices 
float64[10] hbu_pt_bar
float64[6] hpu_pt_bar
float64[12] hbu_tt_celsius
float64 hpu_tt_celsius
float64 hpu_ls_percent
bool[8] hbu_ps
float64 vfd1_energy_kj
float64 vfd1_power_kw
float64 vfd1_speed_rpm
float64 vfd2_energy_kj
float64 vfd2_power_kw
float64 vfd2_speed_rpm
bool heater1_on
bool heater2_on

builtin_interfaces/Time timestamp

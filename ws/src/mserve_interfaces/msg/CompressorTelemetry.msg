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
bool[5] sv                      # Solenoid state
bool[4] ps                      # Booster Position Sensor

uint8 VFD_OFFLINE = 0
uint8 VFD_IDLE    = 1
uint8 VFD_RUNNING = 2
uint8 VFD_FAULT   = 3

uint8[2]   vfd_state            # VFD operating state
float64[2] vfd_speed_rpm        # VFD speed
float64[2] vfd_energy_kj        # VFD Energy
float64[2] vfd_power_kw         # VFD Power
bool heater_on                  # Heater Status — PLC handles both physical heaters in parallel
float64 hpu_tt_celsius          # Hydrolic Tank Temperature
float64 hpu_ls_percent          # Hydrolic Tank Fill Percent

builtin_interfaces/Time timestamp

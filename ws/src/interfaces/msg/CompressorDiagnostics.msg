uint8 OFF = 0
uint8 STARTUP = 1
uint8 AUTO = 2
uint8 MANUAL = 3
uint8 LOCKOUT = 4

builtin_interfaces/Time timestamp
builtin_interfaces/Time telemetry_timestamp
bool telemetry_received
uint8 telemetry_mode

mserve_interfaces/BoosterDiagnostics low_booster
mserve_interfaces/BoosterDiagnostics high_booster

# High-level system state aggregation
# Published by the system model node

uint8 OFF = 0
uint8 STARTUP = 1
uint8 AUTO = 3
uint8 MANUAL = 4
uint8 LOCKOUT = 5

uint8 mode

bool plc_connected
bool system_ready
bool low_booster_available
bool high_booster_available
bool system_faulted
bool system_locked_out

string active_compressor
string status_message

builtin_interfaces/Time timestamp
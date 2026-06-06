uint8 WAIT_COMMAND = 0
uint8 RAMP_VFD_UP = 1
uint8 VFD_DELAY = 2
uint8 WAIT_RAMP = 3
uint8 WAIT_STABILIZATION = 4
uint8 ENERGISE_VALVE = 5
uint8 PCSV_ON = 6
uint8 PCSV_OFF = 7
uint8 DEENERGISE_VALVE = 8
uint8 RAMP_VFD_DOWN = 9
uint8 WAIT_STOP = 10

string name
bool active
bool target_reached
uint8 phase
string status_message

float64 target_pressure_bar
float64 input_pressure_bar
float64 output_pressure_bar
float64 compression_ratio
float64 input_temperature_celsius
float64 output_temperature_celsius
float64 vfd_energy_kj
float64 vfd_speed_rpm
int32 pcsv_cpm
float64 hours_run

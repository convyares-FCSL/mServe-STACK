# mserve_sensehat debug/status snapshot — everything the web UI's
# sensehat.html needs in one message rather than N single-purpose topics.
# Raw sensor_msgs/Imu (SI units, m/s^2 / rad/s) is still published
# separately on ~/imu for anything that wants the ROS-standard message;
# the fields below use the sensor's native/human-friendly units instead,
# for at-a-glance debug reading.

bool online

bool imu_available
bool pressure_available
bool humidity_available
bool led_available
bool joystick_available

float32 temperature_c
float32 pressure_hpa
float32 humidity_percent

geometry_msgs/Vector3 accel_g
geometry_msgs/Vector3 gyro_dps
geometry_msgs/Vector3 mag_ut
float32 heading_deg   # tilt/mag-compensated yaw from RTIMULib's fusion filter, 0-360

# RTIMU::getCompassCalibrationValid()/getAccelCalibrationValid() — whether
# RTIMULib actually has (and is using) calibration data, not just whether
# the sensor is present. false until RTIMULibCal has been run — see
# ws/src/mserve_sensehat/README.md's Calibration section. heading_deg above
# is unreliable while compass_calibrated is false.
bool compass_calibrated
bool accel_calibrated

bool joy_up
bool joy_down
bool joy_left
bool joy_right
bool joy_center

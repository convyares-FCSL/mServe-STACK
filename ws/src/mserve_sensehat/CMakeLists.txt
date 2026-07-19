cmake_minimum_required(VERSION 3.8)
project(mserve_sensehat LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================================================================
# Dependencies
# ==============================================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(utils REQUIRED)

# ==============================================================================
# Vendored RTIMULib (ws/src/third_party/RTIMULib, from
# github.com/RPi-Distro/RTIMULib — MIT-licensed). No apt package exists for
# this distro: librtimulib-dev is Raspberry Pi OS/Debian-only upstream, not
# in Ubuntu's repos (this container's base image).
#
# Built STATIC here (globbed directly, not add_subdirectory-ing its own
# SHARED-library CMakeLists) — same pattern as mserve_lidar's vendored
# rplidar_sdk. Tried the shared-library route first: it built and even ran
# fine under a plain `ros2 run` (LD_LIBRARY_PATH picked up via sourcing this
# package's own local_setup.bash by hand), but failed under the real
# systemd/run_stack.sh launch path with "error while loading shared
# libraries: libRTIMULib.so.7: cannot open shared object file" —
# run_stack.sh deliberately sources only `interfaces`' local_setup.bash, not
# the whole workspace overlay (see docs/TODO.md's Foxglove Bridge note: this
# repo has a package literally named `launch` that shadows the real launch
# framework package if the full overlay is sourced), so no other package's
# install/lib/ ever lands on LD_LIBRARY_PATH at runtime. Static linking
# sidesteps that entirely — no separate runtime library to find.
# ==============================================================================

set(RTIMULIB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/RTIMULib/RTIMULib")
if(NOT EXISTS "${RTIMULIB_SRC_DIR}/RTIMULib.h")
  message(FATAL_ERROR
    "RTIMULib not found at ${RTIMULIB_SRC_DIR} — see ws/src/third_party/README.md's "
    "RTIMULib section to clone it (gitignored, not part of a fresh checkout).")
endif()

# Same file list as RTIMULib/RTIMULib/CMakeLists.txt's LIBRTIMU_SRCS (its
# own build is skipped entirely — only the sources are reused).
add_library(RTIMULib STATIC
  "${RTIMULIB_SRC_DIR}/RTFusion.cpp"
  "${RTIMULIB_SRC_DIR}/RTFusionRTQF.cpp"
  "${RTIMULIB_SRC_DIR}/RTMath.cpp"
  "${RTIMULIB_SRC_DIR}/RTFusionKalman4.cpp"
  "${RTIMULIB_SRC_DIR}/RTIMUAccelCal.cpp"
  "${RTIMULIB_SRC_DIR}/RTIMUHal.cpp"
  "${RTIMULIB_SRC_DIR}/RTIMUMagCal.cpp"
  "${RTIMULIB_SRC_DIR}/RTIMUSettings.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTIMU.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTIMUGD20M303DLHC.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTIMUGD20HM303DLHC.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTIMUGD20HM303D.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTIMULSM9DS0.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTIMULSM9DS1.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTIMUMPU9150.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTIMUMPU9250.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTIMUBMX055.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTIMUBNO055.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTIMUNull.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTPressure.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTPressureBMP180.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTPressureLPS25H.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTPressureMS5611.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTPressureMS5637.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTHumidity.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTHumidityHTS221.cpp"
  "${RTIMULIB_SRC_DIR}/IMUDrivers/RTHumidityHTU21D.cpp"
)
target_include_directories(RTIMULib PUBLIC "$<BUILD_INTERFACE:${RTIMULIB_SRC_DIR}>")

# ==============================================================================
# Main node library
# ==============================================================================

add_library(sensehat_node_lib SHARED
  src/sensehat_node.cpp
  src/sensehat_params.cpp
  src/led_matrix.cpp
  src/joystick_input.cpp
)

target_include_directories(sensehat_node_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE
    "${RTIMULIB_SRC_DIR}"
)

target_link_libraries(sensehat_node_lib
  rclcpp::rclcpp
  std_srvs::std_srvs
  sensor_msgs::sensor_msgs
  interfaces::interfaces
  utils::utils
  RTIMULib
)

# ==============================================================================
# Executable
# ==============================================================================

add_executable(sensehat_node src/main.cpp)
target_include_directories(sensehat_node PRIVATE include)
target_link_libraries(sensehat_node rclcpp::rclcpp)
target_link_libraries(sensehat_node sensehat_node_lib)

# ==============================================================================
# Install
# ==============================================================================

install(TARGETS sensehat_node_lib
  EXPORT export_mserve_sensehat
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS sensehat_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

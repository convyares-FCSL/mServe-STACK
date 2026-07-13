cmake_minimum_required(VERSION 3.8)
project(mserve_lidar LANGUAGES CXX)
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
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(utils REQUIRED)
find_package(Threads REQUIRED)

# ==============================================================================
# Vendored Slamtec RPLIDAR SDK (third_party/rplidar_sdk, from the `ros2` branch
# of github.com/slamtec/rplidar_ros — BSD-licensed, see third_party/rplidar_sdk/LICENSE).
# No apt package exists for this distro (unlike v4l2_camera), and upstream's
# own CMakeLists just globs sdk/src/*.cpp straight into its node executable —
# there's no reusable library target to link against, so it's vendored and
# built the same way here. TCP/UDP channel support and the legacy pre-"sl::"
# A/A2/A3 driver (rplidar_driver.cpp) are dropped — RPLIDAR C1 only ever needs
# the modern sl:: API over a serial channel.
# ==============================================================================

set(RPLIDAR_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/rplidar_sdk")

file(GLOB RPLIDAR_SDK_SRC
  "${RPLIDAR_SDK_DIR}/src/*.cpp"
  "${RPLIDAR_SDK_DIR}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_DIR}/src/dataunpacker/*.cpp"
  "${RPLIDAR_SDK_DIR}/src/dataunpacker/unpacker/*.cpp"
  "${RPLIDAR_SDK_DIR}/src/hal/*.cpp"
)

add_library(rplidar_sdk STATIC ${RPLIDAR_SDK_SRC})
target_include_directories(rplidar_sdk PUBLIC
  "$<BUILD_INTERFACE:${RPLIDAR_SDK_DIR}/include>"
  "$<BUILD_INTERFACE:${RPLIDAR_SDK_DIR}/src>"
)
target_link_libraries(rplidar_sdk PUBLIC Threads::Threads)

# ==============================================================================
# Main node library
# ==============================================================================

add_library(lidar_node_lib SHARED
  src/lidar_node.cpp
  src/lidar_params.cpp
)

target_include_directories(lidar_node_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(lidar_node_lib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  sensor_msgs::sensor_msgs
  utils::utils
  rplidar_sdk
)

# ==============================================================================
# Executable
# ==============================================================================

add_executable(lidar_node src/main.cpp)
target_include_directories(lidar_node PRIVATE include)
target_link_libraries(lidar_node rclcpp::rclcpp)
target_link_libraries(lidar_node lidar_node_lib)

# ==============================================================================
# Install
# ==============================================================================

install(TARGETS lidar_node_lib
  EXPORT export_mserve_lidar
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

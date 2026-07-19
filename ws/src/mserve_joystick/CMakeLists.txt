cmake_minimum_required(VERSION 3.8)
project(mserve_joystick LANGUAGES CXX)
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
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(utils REQUIRED)

# ==============================================================================
# Main node library
# ==============================================================================

add_library(joystick_node_lib SHARED
  src/joystick_node.cpp
  src/joystick_params.cpp
)

target_include_directories(joystick_node_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(joystick_node_lib
  rclcpp::rclcpp
  std_srvs::std_srvs
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
  interfaces::interfaces
  utils::utils
)

# ==============================================================================
# Executable
# ==============================================================================

add_executable(joystick_node src/main.cpp)
target_include_directories(joystick_node PRIVATE include)
target_link_libraries(joystick_node rclcpp::rclcpp)
target_link_libraries(joystick_node joystick_node_lib)

# ==============================================================================
# Install
# ==============================================================================

install(TARGETS joystick_node_lib
  EXPORT export_mserve_joystick
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS joystick_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

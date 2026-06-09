cmake_minimum_required(VERSION 3.8)
project(mserve_drivechain LANGUAGES CXX)
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
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(mserve_interfaces REQUIRED)
find_package(mserve_utils REQUIRED)

# ==============================================================================
# UART + packet layer — no ROS, no BT deps, unit-testable
# ==============================================================================

add_library(drivechain_uart src/drivechain_uart.cpp)
target_include_directories(drivechain_uart PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

# ==============================================================================
# Main node library
# ==============================================================================

add_library(drivechain_node_lib SHARED
  src/drivechain_node.cpp
  src/drivechain_params.cpp
  src/drivechain_bt_nodes.cpp
)

target_include_directories(drivechain_node_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
)

ament_target_dependencies(drivechain_node_lib
  geometry_msgs
  rclcpp
  rclcpp_lifecycle
  behaviortree_cpp
  ament_index_cpp
  lifecycle_msgs
  mserve_interfaces
  mserve_utils
)

target_link_libraries(drivechain_node_lib drivechain_uart)

# ==============================================================================
# Executable
# ==============================================================================

add_executable(drivechain_node src/main.cpp)
target_include_directories(drivechain_node PRIVATE include)
ament_target_dependencies(drivechain_node rclcpp)
target_link_libraries(drivechain_node drivechain_node_lib)

# ==============================================================================
# Install
# ==============================================================================

install(TARGETS drivechain_node_lib drivechain_uart
  EXPORT export_mserve_drivechain
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS drivechain_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY src/trees/
  DESTINATION share/${PROJECT_NAME}/trees
  FILES_MATCHING PATTERN "*.xml"
)

# ==============================================================================
# Tests
# ==============================================================================

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_packet_codec test/test_packet_codec.cpp src/drivechain_uart.cpp)
  target_include_directories(test_packet_codec PRIVATE src include)
endif()

ament_package()

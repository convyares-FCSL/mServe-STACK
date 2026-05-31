cmake_minimum_required(VERSION 3.8)
project(mserve_drivechain LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mserve_interfaces REQUIRED)
find_package(mserve_utils REQUIRED)

# Pure kinematics library — no ROS deps, fully unit-testable
add_library(diff_drive src/diff_drive.cpp)
target_include_directories(diff_drive PUBLIC include)

# Drivechain node
add_executable(drivechain_node
  src/drivechain_node_main.cpp
  src/drivechain_node.cpp
)
ament_target_dependencies(drivechain_node geometry_msgs rclcpp rclcpp_lifecycle mserve_interfaces mserve_utils)
target_include_directories(drivechain_node PRIVATE include)
target_link_libraries(drivechain_node diff_drive)

install(TARGETS drivechain_node diff_drive
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_diff_drive test/test_diff_drive.cpp src/diff_drive.cpp)
  target_include_directories(test_diff_drive PRIVATE include)

  ament_add_gtest(test_packet_codec test/test_packet_codec.cpp)
endif()

ament_package()

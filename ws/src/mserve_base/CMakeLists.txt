cmake_minimum_required(VERSION 3.8)
project(mserve_base LANGUAGES CXX)
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
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(utils REQUIRED)

# ==============================================================================
# Main node library
# ==============================================================================

add_library(base_node_lib SHARED
  src/base_node.cpp
  src/base_params.cpp
  src/base_bt_nodes.cpp
)

target_include_directories(base_node_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
)

target_link_libraries(base_node_lib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  behaviortree_cpp::behaviortree_cpp
  ament_index_cpp::ament_index_cpp
  lifecycle_msgs::lifecycle_msgs
  geometry_msgs::geometry_msgs
  interfaces::interfaces
  utils::utils
)

# ==============================================================================
# Executable
# ==============================================================================

add_executable(base_node src/main.cpp)
target_include_directories(base_node PRIVATE include)
target_link_libraries(base_node rclcpp::rclcpp)
target_link_libraries(base_node base_node_lib)

# ==============================================================================
# Install
# ==============================================================================

install(TARGETS base_node_lib
  EXPORT export_mserve_base
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS base_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY src/trees/
  DESTINATION share/${PROJECT_NAME}/trees
  FILES_MATCHING PATTERN "*.xml"
)

ament_package()

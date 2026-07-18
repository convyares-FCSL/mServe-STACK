cmake_minimum_required(VERSION 3.8)
project(mserve_display LANGUAGES CXX)
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
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(utils REQUIRED)

# ==============================================================================
# Main node library
# ==============================================================================

add_library(display_node_lib SHARED
  src/display_node.cpp
  src/display_params.cpp
  src/framebuffer.cpp
  src/touch_input.cpp
  src/screens.cpp
)

target_include_directories(display_node_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(display_node_lib
  rclcpp::rclcpp
  std_srvs::std_srvs
  lifecycle_msgs::lifecycle_msgs
  geometry_msgs::geometry_msgs
  interfaces::interfaces
  utils::utils
)

# ==============================================================================
# Executable
# ==============================================================================

add_executable(display_node src/main.cpp)
target_include_directories(display_node PRIVATE include)
target_link_libraries(display_node rclcpp::rclcpp)
target_link_libraries(display_node display_node_lib)

# ==============================================================================
# Install
# ==============================================================================

install(TARGETS display_node_lib
  EXPORT export_mserve_display
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS display_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

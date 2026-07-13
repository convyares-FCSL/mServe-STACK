cmake_minimum_required(VERSION 3.8)
project(mserve_camera LANGUAGES CXX)
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

# v4l2_camera::v4l2_camera's exported link interface pulls these in
# transitively but doesn't declare them as CMake config-package dependencies
# of its own — find_package them ourselves first or v4l2_cameraConfig.cmake
# fails with "target not found".
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(v4l2_camera REQUIRED)

# For camera/image_raw/compressed: hand-rolled YUYV -> BGR -> JPEG (cv::cvtColor
# + cv::imencode) rather than image_transport's compressed_image_transport
# plugin — see camera_node.hpp's encode_compressed() comment for why.
find_package(OpenCV REQUIRED)

# ==============================================================================
# Main node library
# ==============================================================================

add_library(camera_node_lib SHARED
  src/camera_node.cpp
  src/camera_params.cpp
)

target_include_directories(camera_node_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(camera_node_lib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  sensor_msgs::sensor_msgs
  v4l2_camera::v4l2_camera
  utils::utils
  ${OpenCV_LIBS}
)

# ==============================================================================
# Executable
# ==============================================================================

add_executable(camera_node src/main.cpp)
target_include_directories(camera_node PRIVATE include)
target_link_libraries(camera_node rclcpp::rclcpp)
target_link_libraries(camera_node camera_node_lib)

# ==============================================================================
# Install
# ==============================================================================

install(TARGETS camera_node_lib
  EXPORT export_mserve_camera
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

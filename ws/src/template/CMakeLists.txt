cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# TODO: rename to hyfleet_<name>
project(hyfleet_subsystem)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================================================================
# Dependencies
# ==============================================================================
# Each package added here must also appear in package.xml as <depend>.

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(mserve_interfaces REQUIRED)
find_package(mserve_utils REQUIRED)
# TODO: add any subsystem-specific deps here (and in package.xml)

# ==============================================================================
# Library
# ==============================================================================

add_library(subsystem_node_lib SHARED
  src/subsystem_node.cpp
  src/subsystem_params.cpp
  src/subsystem_action.cpp
  src/subsystem_bt_nodes.cpp
  # TODO: add src/subsystem_bt_actions.cpp etc. as the package grows
)

target_include_directories(subsystem_node_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
)

ament_target_dependencies(subsystem_node_lib
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  mserve_interfaces
  mserve_utils
  behaviortree_cpp
  behaviortree_ros2
  ament_index_cpp
)

# ==============================================================================
# Executable
# ==============================================================================

# TODO: rename to <name>_node
add_executable(subsystem_node
  src/main.cpp
)

target_include_directories(subsystem_node
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

ament_target_dependencies(subsystem_node rclcpp)

target_link_libraries(subsystem_node subsystem_node_lib)

# ==============================================================================
# Install
# ==============================================================================

install(TARGETS subsystem_node_lib
  EXPORT export_hyfleet_subsystem
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY src/trees/
  DESTINATION share/${PROJECT_NAME}/trees
  FILES_MATCHING PATTERN "*.xml")

install(TARGETS subsystem_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

# ==============================================================================
# Export
# ==============================================================================

ament_export_include_directories(include)
ament_export_libraries(subsystem_node_lib)
ament_export_targets(export_hyfleet_subsystem HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle rclcpp_action mserve_interfaces)

# ==============================================================================
# Tests
# ==============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

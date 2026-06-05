cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(hyfleet_booster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================================================================
# Dependencies
# ==============================================================================
# Each package added here must also appear in package.xml as <depend>.
# Add new deps here first, then in ament_target_dependencies and
# ament_export_dependencies below.

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

# ==============================================================================
# Library — all node logic lives here so it can be tested independently
# ==============================================================================
# Add new .cpp files to this list as the package grows.

add_library(booster_node_lib SHARED
  src/booster_node.cpp
)

# PUBLIC:  include/ — exported to any package that links this library.
#          Only booster_node_lib.hpp lives here (the public API).
# PRIVATE: src/   — internal headers (booster_action.hpp, etc.).
#          Not visible to consumers of the library.
target_include_directories(booster_node_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
)

ament_target_dependencies(booster_node_lib
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  mserve_interfaces
  mserve_utils
)

# ==============================================================================
# Executable — thin wrapper that spins the node
# ==============================================================================

add_executable(booster_node
  src/main.cpp
)

# Executable only needs the public include/ to find booster_node.hpp.
# Internal headers are not needed here — main.cpp just creates the node.
target_include_directories(booster_node
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

ament_target_dependencies(booster_node
  rclcpp
)

target_link_libraries(booster_node
  booster_node_lib
)

# ==============================================================================
# Install
# ==============================================================================

install(TARGETS booster_node_lib
  EXPORT export_hyfleet_booster
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS booster_node
  DESTINATION lib/${PROJECT_NAME}
)

# Public headers exported for downstream packages
install(DIRECTORY include/
  DESTINATION include
)

# ==============================================================================
# Export — allows other packages to link booster_node_lib
# ==============================================================================

ament_export_include_directories(include)
ament_export_libraries(booster_node_lib)
ament_export_targets(export_hyfleet_booster HAS_LIBRARY_TARGET)
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

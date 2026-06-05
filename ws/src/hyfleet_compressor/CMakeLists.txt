cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(hyfleet_compressor)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(mserve_utils REQUIRED)
find_package(mserve_interfaces REQUIRED)

# ==============================================================================
# Library — all node logic lives here so it can be tested independently
# ==============================================================================
# Add new .cpp files to this list as the package grows.

add_library(compressor_node_lib SHARED
  src/compressor_node.cpp
  src/compressor_action.cpp
)

# PUBLIC:  include/ — exported to any package that links this library.
#          Only compressor_node.hpp lives here (the public API).
# PRIVATE: src/   — internal headers (compressor_action.hpp, etc.).
#          Not visible to consumers of the library.
target_include_directories(compressor_node_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
)

ament_target_dependencies(compressor_node_lib
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  mserve_interfaces
  mserve_utils
)

# ==============================================================================
# Executable — thin wrapper that spins the node
# ==============================================================================

add_executable(compressor_node
  src/main.cpp
)

# Executable only needs the public include/ to find compressor_node.hpp.
# Internal headers are not needed here — main.cpp just creates the node.
target_include_directories(compressor_node
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

ament_target_dependencies(compressor_node
  rclcpp
)

target_link_libraries(compressor_node
  compressor_node_lib
)

# ==============================================================================
# Install
# ==============================================================================

install(TARGETS compressor_node_lib
  EXPORT export_hyfleet_compressor
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS compressor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Public headers exported for downstream packages
install(DIRECTORY include/
  DESTINATION include
)

# ==============================================================================
# Export — allows other packages to link compressor_node_lib
# ==============================================================================

ament_export_include_directories(include)
ament_export_libraries(compressor_node_lib)
ament_export_targets(export_hyfleet_compressor HAS_LIBRARY_TARGET)
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

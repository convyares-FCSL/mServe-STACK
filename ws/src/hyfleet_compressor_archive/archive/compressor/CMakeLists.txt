cmake_minimum_required(VERSION 3.8)
project(compressor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(interfaces REQUIRED)

add_library(compressor_node_lib SHARED
  src/booster.cpp
  src/booster_manager.cpp
  src/compressor_action.cpp
  src/compressor_config.cpp
  src/compressor_node.cpp
  src/diagnostics_publisher.cpp
  src/device_dispatcher.cpp
  src/safety_monitor.cpp
  src/telemetry_subscriber.cpp
)
target_include_directories(compressor_node_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(compressor_node_lib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  interfaces
)

add_executable(compressor_main
  src/main.cpp
)
target_include_directories(compressor_main PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(compressor_main
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  interfaces
)
target_link_libraries(compressor_main
  compressor_node_lib
)

install(TARGETS compressor_node_lib
  EXPORT export_compressor
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS compressor_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(compressor_node_lib)
ament_export_targets(export_compressor HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action rclcpp_lifecycle interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

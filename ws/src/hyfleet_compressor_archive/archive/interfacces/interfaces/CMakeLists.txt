cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ControlCompressor.action"
  "msg/BoosterCmd.msg"
  "msg/BoosterDiagnostics.msg"
  "msg/DispenserCmd.msg"
  "msg/GasRouterCmd.msg"
  "msg/GasRouterPath.msg"
  "msg/HResult.msg"
  "msg/CompressorCmd.msg"
  "msg/CompressorDiagnostics.msg"
  "msg/CompressorTelemetry.msg"
  "msg/SystemState.msg"
  "srv/BoosterCmd.srv"
  "srv/CompressorCmd.srv"
  "srv/DispenserCmd.srv"
  "srv/GasRouterCmd.srv"
  DEPENDENCIES builtin_interfaces
)

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

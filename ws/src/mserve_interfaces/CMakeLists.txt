cmake_minimum_required(VERSION 3.8)
project(mserve_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ControlBooster.action"
  "action/ControlCompressor.action"
  "action/Dock.action"

  "msg/BoosterDiagnostics.msg"
   "msg/GasRouterPath.msg"
  "msg/HResult.msg"
  "msg/CompressorDiagnostics.msg"
  "msg/CompressorTelemetry.msg"
  "msg/SystemState.msg"
  "msg/DriveStatus.msg"
  "msg/WheelCommand.msg"
  "msg/WheelFeedback.msg"
  "msg/DisplayStatus.msg"
  "msg/Esp32Status.msg"

  "srv/BoosterCmd.srv"
  "srv/CompressorCmd.srv"
  "srv/DispenserCmd.srv"
  "srv/GasRouterCmd.srv"
  "srv/SetDisplayMode.srv"
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

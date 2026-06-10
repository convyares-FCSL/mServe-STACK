cmake_minimum_required(VERSION 3.8)
project(interfaces)

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

  # Drivechain v2 — per-motor command + feedback
  "msg/MotorCommand.msg"
  "msg/MotorCommands.msg"
  "msg/MotorState.msg"
  "msg/DriveMotorFeedback.msg"

  "srv/BoosterCmd.srv"
  "srv/CompressorCmd.srv"
  "srv/DispenserCmd.srv"
  "srv/DriveChainCmd.srv"
  "srv/GasRouterCmd.srv"
  "srv/SetDisplayMode.srv"
  "srv/SetMode.srv"
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

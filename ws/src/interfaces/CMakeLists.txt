cmake_minimum_required(VERSION 3.8)
project(interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Dock.action"

  "msg/DriveStatus.msg"
  "msg/DisplayStatus.msg"
  "msg/Esp32Status.msg"

  # Drivechain v2 — per-motor command + feedback
  "msg/MotorCommand.msg"
  "msg/MotorState.msg"
  "msg/DriveMotorFeedback.msg"

  "srv/Drive.srv"
  "srv/SetDisplayMode.srv"
  "srv/SetMotorId.srv"
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

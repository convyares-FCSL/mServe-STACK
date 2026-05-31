cmake_minimum_required(VERSION 3.8)
project(mserve_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriveStatus.msg"
  "msg/WheelCommand.msg"
  "msg/WheelFeedback.msg"
  "msg/DisplayStatus.msg"
  "msg/Esp32Status.msg"
  "srv/SetDisplayMode.srv"
  "action/Dock.action"
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

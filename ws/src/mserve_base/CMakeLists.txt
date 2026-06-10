cmake_minimum_required(VERSION 3.8)
project(mserve_base LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(utils REQUIRED)

add_executable(base_node src/base_node_main.cpp src/base_node.cpp)
ament_target_dependencies(base_node geometry_msgs rclcpp rclcpp_lifecycle utils)
target_include_directories(base_node PRIVATE include)

install(TARGETS base_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

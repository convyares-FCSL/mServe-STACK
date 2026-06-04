cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(mserve_lifecycle_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake  REQUIRED)
find_package(mserve_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
# find_package(ZeroMQ REQUIRED)

# include_directories(include)
add_executable(lifecycle_manager  src/main.cpp src/lifecycle_manager.cpp)

# add_executable(lifecycle_managersrc/main.cpp src/bringup_bt.cpp)
ament_target_dependencies(
  lifecycle_manager
  rclcpp
  behaviortree_cpp
  ament_index_cpp
  lifecycle_msgs
  behaviortree_ros2
  mserve_utils
)

# include_directories(include)
target_include_directories(lifecycle_manager PRIVATE include)

install(DIRECTORY src/trees/
  DESTINATION share/${PROJECT_NAME}/trees
  FILES_MATCHING PATTERN "*.xml")

install(TARGETS lifecycle_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(mserve_bringup_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake  REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# include_directories(include)
add_executable(bringup_bt  src/main.cpp )

# add_executable(bringup_bt src/main.cpp src/bringup_bt.cpp)
ament_target_dependencies(
  bringup_bt 
  rclcpp
  behaviortree_cpp
  ament_index_cpp
  lifecycle_msgs
)

# include_directories(include)
target_include_directories(bringup_bt  PRIVATE include)

install(DIRECTORY src/trees/
  DESTINATION share/${PROJECT_NAME}/trees
  FILES_MATCHING PATTERN "*.xml")

install(TARGETS bringup_bt 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

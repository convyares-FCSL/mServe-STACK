cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(hyfleet_compression)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mserve_utils REQUIRED)

add_executable(compression_node src/main.cpp src/compression_node.cpp)
ament_target_dependencies(compression_node rclcpp rclcpp_lifecycle mserve_utils)
target_include_directories(compression_node PRIVATE include)


# include_directories(include)
target_include_directories(compression_node PRIVATE include)


install(TARGETS compression_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()



cmake_minimum_required(VERSION 3.8)
project(launch LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_descriptions test/test_launch_descriptions.py)
endif()

ament_package()

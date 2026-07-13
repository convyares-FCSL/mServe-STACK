cmake_minimum_required(VERSION 3.8)
project(mserve_description LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params rviz urdf worlds models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(test_mserve_description test/test_urdf_load.py)
  add_launch_test(test/test_tf_smoke.py)
endif()

ament_package()

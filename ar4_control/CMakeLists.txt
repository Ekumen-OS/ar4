cmake_minimum_required(VERSION 3.7)
project(ar4_control)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

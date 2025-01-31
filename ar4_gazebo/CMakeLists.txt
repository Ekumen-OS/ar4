cmake_minimum_required(VERSION 3.7)
project(ar4_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    urdf
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()

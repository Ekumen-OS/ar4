cmake_minimum_required(VERSION 3.7)
project(ar4_isaac)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    usda
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()

cmake_minimum_required(VERSION 3.7)
project(ar4_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config
            launch
            meshes
            urdf
  DESTINATION share/${PROJECT_NAME}/)

ament_package()

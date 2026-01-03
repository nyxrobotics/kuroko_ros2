cmake_minimum_required(VERSION 3.8)
project(kuroko_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  xacro
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


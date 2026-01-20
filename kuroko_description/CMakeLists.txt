cmake_minimum_required(VERSION 3.8)
project(kuroko_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  xacro
  meshes
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gz_sim_resource_path.dsv"
)

ament_package()

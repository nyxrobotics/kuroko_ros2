cmake_minimum_required(VERSION 3.8)
project(kuroko_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Python-based ROS 2 nodes (scripts)
install(PROGRAMS
  scripts/spawn_entity_from_param.py
  scripts/unpause_physics.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

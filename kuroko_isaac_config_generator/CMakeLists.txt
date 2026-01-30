cmake_minimum_required(VERSION 3.8)
project(kuroko_isaac_config_generator)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts (ROS 2 "libexec" style)
install(PROGRAMS
  src/sdf_to_usda/sdf_to_usda_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Isaac export script into share so node can run it by default
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

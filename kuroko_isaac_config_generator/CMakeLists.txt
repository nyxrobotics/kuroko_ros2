cmake_minimum_required(VERSION 3.16)
project(kuroko_isaac_config_generator)

find_package(ament_cmake REQUIRED)

# Install Python node script into libexec so ros2 launch can find it.
install(PROGRAMS
  src/sdf_to_usda/sdf_to_usda_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

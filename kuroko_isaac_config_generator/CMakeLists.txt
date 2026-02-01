cmake_minimum_required(VERSION 3.8)
project(kuroko_isaac_config_generator)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

ament_package()

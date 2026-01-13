cmake_minimum_required(VERSION 3.8)
project(kuroko_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(imu_drift_remover_component SHARED
  src/imu_drift_remover.cpp
)
target_include_directories(imu_drift_remover_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(imu_drift_remover_component
  rclcpp rclcpp_components sensor_msgs std_msgs
)
rclcpp_components_register_nodes(imu_drift_remover_component
  "kuroko_imu::ImuDriftRemover"
)

add_executable(imu_drift_remover
  src/main.cpp
)
target_link_libraries(imu_drift_remover imu_drift_remover_component)
ament_target_dependencies(imu_drift_remover rclcpp)

install(TARGETS
  imu_drift_remover_component
  imu_drift_remover
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

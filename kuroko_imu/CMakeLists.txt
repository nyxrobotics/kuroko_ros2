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
find_package(tf2 REQUIRED)

# IMU drift remover 
add_executable(imu_drift_remover
  src/imu_drift_remover/main.cpp
  src/imu_drift_remover/imu_drift_remover.cpp
)

target_include_directories(imu_drift_remover PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(imu_drift_remover
  rclcpp rclcpp_components sensor_msgs std_msgs
)

# IMU angular velocity overwriter
add_executable(imu_angular_velocity_overwriter
  src/imu_angular_velocity_overwriter/main.cpp
  src/imu_angular_velocity_overwriter/imu_angular_velocity_overwriter.cpp
)
ament_target_dependencies(imu_angular_velocity_overwriter
  rclcpp sensor_msgs tf2
)
target_include_directories(imu_angular_velocity_overwriter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# IMU accel low-pass filter (norm + direction)
add_executable(imu_accel_lpf
  src/imu_accel_lpf/main.cpp
  src/imu_accel_lpf/imu_accel_lpf.cpp
)
ament_target_dependencies(imu_accel_lpf
  rclcpp sensor_msgs tf2
)
target_include_directories(imu_accel_lpf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets and files
install(TARGETS
  imu_drift_remover
  imu_angular_velocity_overwriter
  imu_accel_lpf
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

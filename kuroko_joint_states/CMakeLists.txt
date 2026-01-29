cmake_minimum_required(VERSION 3.8)
project(kuroko_joint_states)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joint_states_velocity_overwriter src/joint_states_velocity_overwriter.cpp)
ament_target_dependencies(joint_states_velocity_overwriter rclcpp sensor_msgs)

install(TARGETS
  joint_states_velocity_overwriter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

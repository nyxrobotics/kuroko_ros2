cmake_minimum_required(VERSION 3.8)
project(kuroko_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kuroko_kinematics REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(foot_odom_node
  src/foot_odom_node.cpp
)
ament_target_dependencies(foot_odom_node
  rclcpp sensor_msgs nav_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs
  kuroko_kinematics
)
target_link_libraries(foot_odom_node Eigen3::Eigen)

add_executable(odom_tf_node
  src/odom_tf_node.cpp
)
ament_target_dependencies(odom_tf_node
  rclcpp nav_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs
)
target_link_libraries(odom_tf_node Eigen3::Eigen)

install(TARGETS
  foot_odom_node
  odom_tf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

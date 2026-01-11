cmake_minimum_required(VERSION 3.8)
project(kuroko_kinematics)

# =========================
# Compile options
# =========================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =========================
# Dependencies
# =========================
find_package(ament_cmake REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# =========================
# Include directories
# =========================
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# =========================
# Core kinematics library
# =========================
add_library(${PROJECT_NAME}
  src/kuroko_kinematics.cpp
  src/joint_data.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  Eigen3
)

# =========================
# FK–IK consistency checker node
# =========================
add_executable(fk_ik_consistency_checker
  src/fk_ik_consistency_checker.cpp
)

target_link_libraries(fk_ik_consistency_checker
  ${PROJECT_NAME}
)

ament_target_dependencies(fk_ik_consistency_checker
  rclcpp
  sensor_msgs
  geometry_msgs
)

# =========================
# Install
# =========================
install(
  TARGETS
    ${PROJECT_NAME}
    fk_ik_consistency_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(
    DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# =========================
# Export
# =========================
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)

ament_package()

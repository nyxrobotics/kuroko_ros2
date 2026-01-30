cmake_minimum_required(VERSION 3.8)
project(kuroko_isaac_config_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDFORMAT12 REQUIRED sdformat12)

# -----------------------------
# URDF -> SDF helper executable
# -----------------------------
add_executable(urdf_to_sdf
  src/usda_generator/urdf_to_sdf/urdf_to_sdf.cpp
)
target_include_directories(urdf_to_sdf PRIVATE
  include
  ${SDFORMAT12_INCLUDE_DIRS}
)
target_link_directories(urdf_to_sdf PRIVATE ${SDFORMAT12_LIBRARY_DIRS})
target_link_libraries(urdf_to_sdf ${SDFORMAT12_LIBRARIES})
target_compile_options(urdf_to_sdf PRIVATE ${SDFORMAT12_CFLAGS_OTHER})

install(TARGETS urdf_to_sdf
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------
# usda_generator node (Python)
# Install the node script and its helper modules into the same directory
# so the node can do simple local imports.
# -----------------------------
install(PROGRAMS
  src/usda_generator/usda_generator_node.py
  DESTINATION lib/${PROJECT_NAME}/usda_generator
)

install(FILES
  src/usda_generator/xacro_to_urdf.py
  src/usda_generator/ros_package_path.py
  src/usda_generator/sdf_normalizer.py
  src/usda_generator/isaac_usd_export.py
  src/usda_generator/isaac_usd_export_script.py
  DESTINATION lib/${PROJECT_NAME}/usda_generator
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY notebooks/
  DESTINATION share/${PROJECT_NAME}/notebooks
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

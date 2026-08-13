cmake_minimum_required(VERSION 3.8)
project(robot_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install only directories that exist in the package
install(
  DIRECTORY config launch meshes urdf scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install executable Python scripts to the 'lib' folder
install(
  PROGRAMS 
    scripts/controller.py 
    scripts/odom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
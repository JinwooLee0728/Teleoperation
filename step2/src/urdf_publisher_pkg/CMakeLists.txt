
cmake_minimum_required(VERSION 3.10)
project(urdf_publisher_pkg)

find_package(ament_cmake REQUIRED)

# Install URDF files
install(
  DIRECTORY /home/jinwoo/Desktop/teleop_prj/FACTR_materials/FACTR_Hardware/urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


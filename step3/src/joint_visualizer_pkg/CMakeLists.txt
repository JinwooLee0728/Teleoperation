cmake_minimum_required(VERSION 3.8)
project(joint_visualizer_pkg)

# ----------------------------
# Compiler settings
# ----------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------
# Find ROS 2 packages
# ----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ----------------------------
# Find Dynamixel SDK
# ----------------------------
find_package(dynamixel_sdk REQUIRED)

# ----------------------------
# Build executable
# ----------------------------
add_executable(joint_state_publisher_node
  src/joint_visualizer.cpp
)

ament_target_dependencies(joint_state_publisher_node
  rclcpp
  sensor_msgs
  dynamixel_sdk
)

# ----------------------------
# Install
# ----------------------------
install(TARGETS
  joint_state_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# ----------------------------
# Package export
# ----------------------------
ament_package()


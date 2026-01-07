cmake_minimum_required(VERSION 3.8)
project(wrench_estimator_pkg)

# ----------------------------
# Compiler settings
# ----------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------
# Find ROS 2 packages
# ----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ----------------------------
# Find Dynamixel SDK
# ----------------------------
find_package(dynamixel_sdk REQUIRED)

# ----------------------------
# Find Pinocchio (from ROS)
# ----------------------------
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# ----------------------------
# Build executable
# ----------------------------
add_executable(wrench_estimator_node
  src/position_and_wrench_node.cpp
  src/wrench_estimator.cpp
)

ament_target_dependencies(wrench_estimator_node
  rclcpp
  sensor_msgs
  geometry_msgs
  dynamixel_sdk
  pinocchio
)

target_link_libraries(wrench_estimator_node
  Eigen3::Eigen
  pinocchio::pinocchio
)   # Eigen3 is not ROS related

target_include_directories(wrench_estimator_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


# ----------------------------
# Install
# ----------------------------
install(TARGETS
  wrench_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)


# ----------------------------
# Package export
# ----------------------------
ament_package()
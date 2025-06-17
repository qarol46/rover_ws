cmake_minimum_required(VERSION 3.8)
project(sp_udp_communication)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

add_executable(odometry_fus_node
  src/odometry_fus.cpp
)
ament_target_dependencies(odometry_fus_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS odometry_fus_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclcpp_components
)
ament_export_include_directories(include)
ament_export_libraries(udp_manipulator)

ament_package()
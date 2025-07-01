cmake_minimum_required(VERSION 3.8)
project(odometry_fus)

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
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Узел fusion odometry
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

# Узел фильтра частиц
add_executable(particle_filter_no_map_node
  src/particle_filter_no_map.cpp
)
ament_target_dependencies(particle_filter_no_map_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  tf2_msgs
  visualization_msgs
  std_msgs
)

install(TARGETS 
  odometry_fus_node
  particle_filter_no_map_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
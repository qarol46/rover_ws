cmake_minimum_required(VERSION 3.8)
project(rover_navigation)

cmake_policy(SET CMP0074 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation sample_consensus search kdtree features)

# Include directories
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# Ground filter node
add_executable(ground_filter_node src/ground_filter_node.cpp)

# Link libraries
target_link_libraries(ground_filter_node
  ${PCL_LIBRARIES}
)

# ROS 2 dependencies
ament_target_dependencies(ground_filter_node
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
)

# Install targets and directories
install(TARGETS ground_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch 
  config 
  maps 
  rviz 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
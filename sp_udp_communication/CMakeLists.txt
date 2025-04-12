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

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# UDP library
add_library(udp_manipulator
  src/udp_manipulator.cpp
)
ament_target_dependencies(udp_manipulator
  rclcpp
)
target_include_directories(udp_manipulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Robot server node
add_executable(udp_robot_server
  src/udp_robot_server.cpp
)
ament_target_dependencies(udp_robot_server
  rclcpp
  rclcpp_components
)
target_link_libraries(udp_robot_server
  udp_manipulator
)

# Controller client node
add_executable(udp_controller_client
  src/udp_controller_client.cpp
)
ament_target_dependencies(udp_controller_client
  rclcpp
  rclcpp_components
)
target_link_libraries(udp_controller_client
  udp_manipulator
)

# Install
install(TARGETS
  udp_manipulator
  udp_robot_server
  udp_controller_client
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
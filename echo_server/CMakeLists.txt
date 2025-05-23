cmake_minimum_required(VERSION 3.8)
project(echo_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(plat_part src/plat_server.cpp)

ament_target_dependencies(plat_part
  rclcpp
  geometry_msgs
  tf2_ros
)

install(TARGETS
  plat_part
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  ros2_control_wheeled_robot_hardware
)
ament_package()

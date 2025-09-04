cmake_minimum_required(VERSION 3.5)
project(real_rover)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ros2_control_wheeled_robot_hardware REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(bluespace_ai_xsens_mti_driver REQUIRED)

install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  ros2_control_wheeled_robot_hardware
  rclcpp_lifecycle  # Добавлено в экспортируемые зависимости
)

ament_package()
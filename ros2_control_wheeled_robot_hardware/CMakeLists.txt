cmake_minimum_required(VERSION 3.5)
project(ros2_control_wheeled_robot_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)  # Добавлена новая зависимость

# Основная библиотека
add_library(ros2_control_wheeled_robot_hardware SHARED
  src/wheeled_robot_hardware.cpp
  src/udp_wheeled_robot.cpp
)

target_include_directories(ros2_control_wheeled_robot_hardware
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ros2_control_wheeled_robot_hardware
  hardware_interface
  rclcpp
  pluginlib
  rclcpp_lifecycle  # Добавлено в зависимости цели
)

# Тестовая нода для проверки UDP
add_executable(udp_test_node 
  src/udp_test_node.cpp
)

target_link_libraries(udp_test_node
  ros2_control_wheeled_robot_hardware
)

ament_target_dependencies(udp_test_node
  rclcpp
)

install(TARGETS 
  ros2_control_wheeled_robot_hardware
  udp_test_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

ament_export_libraries(ros2_control_wheeled_robot_hardware)
ament_export_dependencies(
  hardware_interface
  rclcpp
  pluginlib
  rclcpp_lifecycle  # Добавлено в экспортируемые зависимости
)

ament_package()
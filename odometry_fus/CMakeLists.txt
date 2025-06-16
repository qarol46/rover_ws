cmake_minimum_required(VERSION 3.8)
project(odometry_fus)

# Найти зависимости
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Создать исполняемый файл
add_executable(odometry_fus_node src/odometry_fus.cpp)

# Подключить зависимости
ament_target_dependencies(odometry_fus_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Установить
install(TARGETS odometry_fus_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
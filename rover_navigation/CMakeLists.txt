cmake_minimum_required(VERSION 3.5)
project(rover_navigation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common segmentation filters)

# Сообщения для отладки
message(STATUS "PCL include dirs: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL libraries: ${PCL_LIBRARIES}")

# Ground filter node
add_executable(ground_filter_node src/ground_filter_node.cpp)
target_include_directories(ground_filter_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(ground_filter_node
  ${PCL_LIBRARIES}
)
ament_target_dependencies(
  ground_filter_node
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

install(
  DIRECTORY launch rviz config maps
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ground_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
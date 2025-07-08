cmake_minimum_required(VERSION 3.5)
project(rover_navigation)
cmake_policy(SET CMP0074 NEW)

# Default to C++17 for better features and type safety
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=sign-compare)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common segmentation filters)

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
  tf2_eigen  
  pcl_conversions
)

# Install targets and resources
install(TARGETS ground_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz config maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
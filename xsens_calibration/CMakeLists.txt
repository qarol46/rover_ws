cmake_minimum_required(VERSION 3.8)
project(xsens_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# add the executable 
add_executable(xsens_calib src/xsens_calibration.cpp)
ament_target_dependencies(xsens_calib rclcpp std_msgs sensor_msgs)

# find the executable 
install(TARGETS
  xsens_calib
  DESTINATION lib/${PROJECT_NAME})

ament_package()

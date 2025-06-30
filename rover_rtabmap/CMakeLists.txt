cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0074 NEW)
project(rover_rtabmap)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
   DESTINATION share/${PROJECT_NAME}
)

ament_package()
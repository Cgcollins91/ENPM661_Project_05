# Project: ENPM662-Project1-Group1
# License: MIT
# The code in this file represents the collective work of Group 1.
# At times, code has been liberally borrowed from files provided
# with the project's instructions or from OSRF training materials.
# Please see the project report for a list of references, or contact
# the team with specific questions.

cmake_minimum_required(VERSION 3.8)
project(terp2_controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

add_executable(terp2_controller
        src/terp2_controller.h src/terp2_controller.cpp
        src/quaternion.h src/quaternion.cpp 
)

ament_target_dependencies(terp2_controller rclcpp geometry_msgs nav_msgs sensor_msgs gazebo_msgs)

install(TARGETS terp2_controller DESTINATION lib/${PROJECT_NAME})

ament_package()

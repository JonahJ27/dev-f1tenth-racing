cmake_minimum_required(VERSION 3.5)
project(cpp_gap_follow)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(rgf src/reactive_gap_follow.cpp)
ament_target_dependencies(rgf rclcpp sensor_msgs ackermann_msgs)

install(TARGETS
  rgf
  DESTINATION lib/${PROJECT_NAME})

ament_package()
cmake_minimum_required(VERSION 3.5)
project(cpp_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(safety src/aeb.cpp)
ament_target_dependencies(safety rclcpp nav_msgs sensor_msgs ackermann_msgs)

install(TARGETS
  safety
  DESTINATION lib/${PROJECT_NAME})

ament_package()
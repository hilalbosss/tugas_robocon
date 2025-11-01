cmake_minimum_required(VERSION 3.5)
project(turtle_circle_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(circle_mover src/circle_mover.cpp)
ament_target_dependencies(circle_mover rclcpp geometry_msgs turtlesim)

install(TARGETS
  circle_mover
  DESTINATION lib/${PROJECT_NAME})

ament_package()

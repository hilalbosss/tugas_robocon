cmake_minimum_required(VERSION 3.8)
project(turtle_flower_cpp)

# Jika pakai GCC, aktifkan warning yang berguna
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Temukan dependency utama
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# Tambahkan executable utama
add_executable(flower_drawer src/flower_drawer.cpp)

# Hubungkan executable dengan library ROS2
ament_target_dependencies(flower_drawer
  rclcpp
  geometry_msgs
  turtlesim
)

# Tentukan tempat instalasi binary
install(TARGETS
  flower_drawer
  DESTINATION lib/${PROJECT_NAME}
)

# Maklumat untuk build ROS2
ament_package()

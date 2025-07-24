cmake_minimum_required(VERSION 3.5)
project(jetracer)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# C++ deps
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_executable(jetracer src/jetracer.cpp)
ament_target_dependencies(jetracer
  rclcpp sensor_msgs nav_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs Boost
)

install(TARGETS jetracer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/teleop_joy.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

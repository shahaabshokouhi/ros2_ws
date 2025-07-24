cmake_minimum_required(VERSION 3.8)
project(orb_slam2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(std_msgs     REQUIRED)
find_package(sensor_msgs  REQUIRED)
find_package(cv_bridge    REQUIRED)
find_package(OpenCV       REQUIRED)

# Find your installed ORB-SLAM2
find_path   (ORB_SLAM2_INCLUDE_DIR  NAMES System.h       PATHS /usr/local/include/ORB_SLAM2)
find_library(ORB_SLAM2_LIB         NAMES ORB_SLAM2       PATHS /usr/local/lib)

# Make sure the headers are visible
include_directories(
  ${ORB_SLAM2_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# 1) Define the executable target
add_executable(orb_slam2_node src/orb_slam2_pose.cpp)

# 2) Declare its ROS 2 dependencies
ament_target_dependencies(
  orb_slam2_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

# 3) Link in ORB-SLAM2 and OpenCV libraries
target_link_libraries(orb_slam2_node
  ${ORB_SLAM2_LIB}
  ${OpenCV_LIBRARIES}
)

# 4) Install it
install(TARGETS orb_slam2_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

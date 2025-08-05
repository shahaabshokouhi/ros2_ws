cmake_minimum_required(VERSION 3.8)
project(orb_slam2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 and system dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(realsense2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)

# Include ORB-SLAM2 headers
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  /home/jetson1/ORB_SLAM2/include
  /home/jetson1/ORB_SLAM2/Thirdparty/DBoW2
  /home/jetson1/ORB_SLAM2/Thirdparty/DBoW2/DBoW2     # ← add this
  /home/jetson1/ORB_SLAM2/Thirdparty/g2o
  /home/jetson1/ORB_SLAM2/Thirdparty/g2o/g2o
  /home/jetson1/ORB_SLAM2                                 # ← NEW: Root path 
)

# Add your ORB-SLAM2 library
link_directories(/home/jetson1/ORB_SLAM2/lib)

# Define the executable node
add_executable(orb_slam_node src/orb_slam_node.cpp)

# Declare its ROS 2 dependencies

ament_target_dependencies(
  orb_slam_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
  tf2
  tf2_ros
)

# Link libraries
target_link_libraries(orb_slam_node
  ORB_SLAM2
  ${OpenCV_LIBRARIES}
  ${Pangolin_LIBRARIES}
)

install(
  DIRECTORY /home/jetson1/ORB_SLAM2/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

# Install node
install(TARGETS orb_slam_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
 )

# Optional: enable tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
add_compile_options(-Wno-deprecated-declarations)

ament_package()

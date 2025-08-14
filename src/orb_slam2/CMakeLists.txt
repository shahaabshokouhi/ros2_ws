cmake_minimum_required(VERSION 3.8)
project(orb_slam2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-declarations)
endif()

# ========== ROS 2 Dependencies ==========
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
find_package(orbslam2_msgs REQUIRED)

# ========== ORB_SLAM2 Local Build Paths ==========
if(NOT DEFINED ENV{ORB_SLAM2_ROOT})
  message(FATAL_ERROR "\n\n ORB_SLAM2_ROOT environment variable is not defined.\nPlease set it in your shell, e.g.:\n\n    export ORB_SLAM2_ROOT=/home/jetson1/ORB_SLAM2\n\nThen run:\n\n    source ~/.bashrc\n    colcon build --packages-select your_package\n")
endif()

set(ORB_SLAM2_ROOT $ENV{ORB_SLAM2_ROOT})

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${ORB_SLAM2_ROOT}
  ${ORB_SLAM2_ROOT}/include
  ${ORB_SLAM2_ROOT}/Thirdparty/DBoW2
  ${ORB_SLAM2_ROOT}/Thirdparty/DBoW2/DBoW2
  ${ORB_SLAM2_ROOT}/Thirdparty/g2o
  ${ORB_SLAM2_ROOT}/Thirdparty/g2o/g2o
)

link_directories(
  ${ORB_SLAM2_ROOT}/lib
  ${ORB_SLAM2_ROOT}/Thirdparty/DBoW2/lib
  ${ORB_SLAM2_ROOT}/Thirdparty/g2o/lib
)

# ========== ROS 2 Node ==========
add_executable(orb_slam_node src/orb_slam_node.cpp)
add_executable(map_receiver src/map_receiver.cpp)

ament_target_dependencies(
  orb_slam_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  cv_bridge
  image_transport
  message_filters
  orbslam2_msgs
)

ament_target_dependencies(
  map_receiver
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  cv_bridge
  image_transport
  message_filters
  orbslam2_msgs
)

target_link_libraries(orb_slam_node
  ORB_SLAM2
  DBoW2
  ${OpenCV_LIBRARIES}
  ${Pangolin_LIBRARIES}
)

target_link_libraries(map_receiver
  ORB_SLAM2
  DBoW2
  ${OpenCV_LIBRARIES}
  ${Pangolin_LIBRARIES}
)

set_target_properties(orb_slam_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${ORB_SLAM2_ROOT}/lib"
)

set_target_properties(map_receiver PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${ORB_SLAM2_ROOT}/lib"
)
# ========== Install Targets ==========
install(TARGETS orb_slam_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS map_receiver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ========== Packaging ==========
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(orb_slam3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-declarations)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========== ROS 2 Dependencies ==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(orbslam2_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)

# ========== ORB_SLAM3 Local Build Paths ==========
# Fully self-contained tree: its own Pangolin/DBoW2/g2o/Sophus live inside it,
# so this package shares NOTHING with the ORB_SLAM2 setup.
if(NOT DEFINED ENV{ORB_SLAM3_ROOT})
  message(FATAL_ERROR "\n\n ORB_SLAM3_ROOT environment variable is not defined.\nPlease set it in your shell, e.g.:\n\n    export ORB_SLAM3_ROOT=/home/shahab/ORB_SLAM3\n\nThen run:\n\n    source ~/.bashrc\n    colcon build --packages-select orb_slam3\n")
endif()

set(ORB_SLAM3_ROOT $ENV{ORB_SLAM3_ROOT})

# Use the Pangolin built privately inside the ORB_SLAM3 tree (never the
# system one, and never the ORB_SLAM2 copy). Pangolin v0.6 places its
# config file under build/src.
set(Pangolin_DIR ${ORB_SLAM3_ROOT}/Thirdparty/Pangolin/build/src)
find_package(Pangolin REQUIRED PATHS ${Pangolin_DIR} NO_DEFAULT_PATH)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${ORB_SLAM3_ROOT}
  ${ORB_SLAM3_ROOT}/include
  ${ORB_SLAM3_ROOT}/include/CameraModels
  ${ORB_SLAM3_ROOT}/Thirdparty
  ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2
  ${ORB_SLAM3_ROOT}/Thirdparty/g2o
  ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
)

link_directories(
  ${ORB_SLAM3_ROOT}/lib
  ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib
  ${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib
)

# ========== ROS 2 Node ==========
add_executable(orb_slam3_node src/orb_slam3_node.cpp)

ament_target_dependencies(
  orb_slam3_node
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

target_link_libraries(orb_slam3_node
  ORB_SLAM3
  DBoW2
  g2o
  ${OpenCV_LIBRARIES}
  ${Pangolin_LIBRARIES}
  Boost::serialization
  OpenSSL::Crypto
)

set_target_properties(orb_slam3_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib;${ORB_SLAM3_ROOT}/Thirdparty/Pangolin/build/src"
)

# ========== Install Targets ==========
install(TARGETS orb_slam3_node
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

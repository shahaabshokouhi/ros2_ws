#!/usr/bin/env bash
# Wipes build/, install/ and log/ of ~/ros2_ws (an OLD workspace path) and rebuilds only the
# ORB-SLAM2 packages (orbslam2_msgs, orb_slam2) with --symlink-install. It does not build
# orb_slam3, jetracer, jetracer_sim or ma_explore_ros: use colcon build for those.
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select orbslam2_msgs --symlink-install
source install/setup.bash
colcon build --packages-select orb_slam2 --symlink-install
source install/setup.bash

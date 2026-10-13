#!/bin/bash
# OLD ORB-SLAM2 test (paths of the robot 'jetson1' hard-coded): builds the orb_slam2 package and runs
# one node, orb_slam_node_0, with the RealSense settings. Kept for reference. For SLAM use
# run_slam3.sh (ORB-SLAM3, the current one) or run_slam.sh (ORB-SLAM2).
colcon build --packages-select orb_slam2 --cmake-clean-cache
source install/setup.bash
ros2 launch orb_slam2 orb_slam.launch.py \
    node_name:=orb_slam_node_0 \
    vocab_file:=/home/jetson1/ORB_SLAM2/Vocabulary/ORBvoc.txt \
    settings_file:=/home/jetson1/ORB_SLAM2/Examples/RGB-D/realsense.yaml
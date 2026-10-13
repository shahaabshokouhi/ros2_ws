#!/bin/bash
# Robot: builds and runs the ORB-SLAM2 multi-agent node (the earlier SLAM; the current one is
# ORB-SLAM3: run_slam3.sh). Needs AGENT_NAME, ORB_SLAM2_ROOT and REALSENSE_CONFIG.
colcon build --packages-select orb_slam2 --cmake-clean-cache
source install/setup.bash
ros2 launch orb_slam2 orb_slam.launch.py \
    agent:="$AGENT_NAME" \
    vocab_file:=${ORB_SLAM2_ROOT}/Vocabulary/ORBvoc.txt \
    settings_file:="$REALSENSE_CONFIG"

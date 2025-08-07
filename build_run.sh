colcon build --packages-select orb_slam2 --cmake-clean-cache
source install/setup.bash
ros2 launch orb_slam2 orb_slam.launch.py \
    node_name:=orb_slam_node_0 \
    vocab_file:=/home/jetson1/ORB_SLAM2/Vocabulary/ORBvoc.txt \
    settings_file:=/home/jetson1/ORB_SLAM2/Examples/RGB-D/realsense.yaml
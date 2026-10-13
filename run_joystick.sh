#!/bin/bash
# Robot: builds the jetracer package and starts manual driving with a joystick: joy_node and
# teleop_joy, the base driver, an EKF (robot_localization) on wheel odometry and IMU, the display
# node and static transforms. Needs AGENT_NAME. It publishes odom -> base_footprint itself, so do
# not combine it with run_slam3.sh --nav / --explore (they take the pose from SLAM).
colcon build --packages-select jetracer --cmake-clean-cache
source install/setup.bash
ros2 launch jetracer jetracer.launch.py \
  use_sim_time:=false agent_name:="$AGENT_NAME"

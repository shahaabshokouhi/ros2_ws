#!/bin/bash
# ROBOT. Manual driving with a joystick: joystick node, base driver, an EKF on wheel
# odometry + IMU, display and static transforms. Builds the jetracer package first.
# Needs AGENT_NAME. Runs alone: it publishes odom -> base_footprint itself, so not
# together with run_slam.sh --nav / --explore (those take the pose from SLAM).
# run_slam.sh without --nav may run beside it.
#
#   ./run_joystick.sh
#
# Example:  ./run_joystick.sh   (then, in a second terminal: ./run_slam.sh)
if [[ "$1" == -h || "$1" == --help ]]; then awk 'NR>1{if(!/^#/)exit; sub(/^# ?/,""); print}' "$0"; exit 0; fi
colcon build --packages-select jetracer --cmake-clean-cache
source install/setup.bash
ros2 launch jetracer jetracer.launch.py \
  use_sim_time:=false agent_name:="$AGENT_NAME"

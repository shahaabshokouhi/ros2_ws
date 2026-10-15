#!/bin/bash
# ROBOT, Vicon lab. Vicon PID waypoint follower (pose from /vicon/AGENT/AGENT, waypoints
# from its YAML file, or goals on a topic with goal_subscriber:=true) and the base driver.
# Builds the jetracer package first. Needs AGENT_NAME and the Vicon bridge running.
# Not together with run_slam.sh --nav / --explore (both would drive the car).
#
#   ./run_controller.sh
#
# Example:  ./run_controller.sh   (with ./run_slam.sh beside it to record SLAM against Vicon)
if [[ "$1" == -h || "$1" == --help ]]; then awk 'NR>1{if(!/^#/)exit; sub(/^# ?/,""); print}' "$0"; exit 0; fi
colcon build --packages-select jetracer --cmake-clean-cache
source install/setup.bash
ros2 launch jetracer pid_controller.launch.py \
  use_sim_time:=false agent_name:="$AGENT_NAME" goal_subscriber:=true


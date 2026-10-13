#!/bin/bash
# Robot, Vicon lab: builds the jetracer package and starts the Vicon PID waypoint follower
# (pid_controller.py: pose from /vicon/<agent>/<agent>, waypoints from its YAML file, or goals on
# a topic with goal_subscriber:=true) and the base driver. Needs AGENT_NAME and the Vicon bridge.
# Do not run it with run_slam3.sh --nav / --explore: both would drive the car.
colcon build --packages-select jetracer --cmake-clean-cache
source install/setup.bash
ros2 launch jetracer pid_controller.launch.py \
  use_sim_time:=false agent_name:="$AGENT_NAME" goal_subscriber:=true


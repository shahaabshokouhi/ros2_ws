#!/bin/bash
# Isaac Sim stand-in for one JetRacer (jetracer_sim). Start this first, then
# ./run_sim_slam.sh in another terminal.
#
#   ./run_isaac_sim.sh [--headless] [--agent sim] [--usd FILE] [--no-realtime] [--plain-room] [--light N]
#
# Uses ROS domain $SIM_DOMAIN (default 31) so the simulated robot never mixes
# with real robots on the lab network; RViz and teleop need the same:
#   ROS_DOMAIN_ID=31 ./run_rviz.sh sim      ROS_DOMAIN_ID=31 ./run_teleop.sh sim
HERE="$(cd "$(dirname "$0")" && pwd)"
source ~/activate_isaaclab_ros2.sh
export ROS_DOMAIN_ID=${SIM_DOMAIN:-31}
echo "Isaac Sim on ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
exec python "$HERE/src/jetracer_sim/isaac/run_isaac.py" "$@"

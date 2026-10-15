#!/bin/bash
# SIMULATION. Starts Isaac Sim with JetRacer cars (jetracer_sim), on ROS domain $SIM_DOMAIN
# (default 31) so it never mixes with real robots. Mostly started by the other simulation
# scripts (run_ma_sim_test.sh, run_ma_sim_nav.sh, run_isaac_explore.sh). By hand: one car,
# then ./run_sim_slam.sh in a second terminal; or just to look at a world.
#
#   ./run_isaac_sim.sh [--world DIR] [--world-light F] [--agents N] [--agent NAME]
#                      [--poses "x,y,yaw;..."] [--headless] [--view CAMERA] [--gt-log DIR]
#                      [--no-realtime] [--usd FILE] [--plain-room] [--light F]
#                      [--width W] [--height H] [--max-steer RAD]
#
#   --world DIR     drive in another world (e.g. ~/isaac_worlds/big_env, the 22 x 16 m lab);
#                   default: the 6 x 6 m room of --usd
#   --world-light F scale that world's lights (default 1)
#   --agents N      number of cars (default 1); names AGENT_1 ... AGENT_N (one car: AGENT)
#   --agent NAME    the name (prefix) of the cars (default sim)
#   --poses         start poses in the world frame, one "x,y,yaw_deg" per car
#   --headless      no window
#   --view CAMERA   window camera: Top, Corner_NE, Corner_NW, Corner_SW (default), Corner_SE
#   --gt-log DIR    write each car's true pose to DIR/AGENT_gt.csv
#   --no-realtime   run as fast as possible
#   --usd FILE      the scene with the car (default ~/Jetracer/jetracer3.usd)
#   --plain-room    the room without its furniture and posters
#   --light F       the room's light intensity
#   --width, --height  camera image size (default 640 x 480)
#   --max-steer RAD steering limit (default 0.59)
#
# Examples:
#   ./run_isaac_sim.sh                     one car in the room (then ./run_sim_slam.sh)
#   ./run_isaac_sim.sh --world ~/isaac_worlds/big_env --agents 3 --poses="-8.5,-3,90;8.5,-3,90;0,6.9,180" --view Top
#                                          look at the lab with the three cars at their starts
if [[ "$1" == -h || "$1" == --help ]]; then awk 'NR>1{if(!/^#/)exit; sub(/^# ?/,""); print}' "$0"; exit 0; fi
HERE="$(cd "$(dirname "$0")" && pwd)"
source ~/activate_isaaclab_ros2.sh
export ROS_DOMAIN_ID=${SIM_DOMAIN:-31}
echo "Isaac Sim on ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
exec python "$HERE/src/jetracer_sim/isaac/run_isaac.py" "$@"

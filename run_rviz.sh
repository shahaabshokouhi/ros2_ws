#!/bin/bash
# PC (station) or simulation. RViz for one robot: occupancy grid, pose, path, Nav2 plan and
# costmaps; give goals with the "2D Goal Pose" button. Shows the gray camera view when
# the robot runs run_slam.sh --teleop. Needs the robot's ROS_DOMAIN_ID (simulation: 31).
# Robot side: ./run_slam.sh --nav (or --grid / --teleop / --explore).
#
#   ./run_rviz.sh [AGENT [FRAME_PREFIX]]
#
#   AGENT          the robot's agent name (default $AGENT_NAME)
#   FRAME_PREFIX   the robot's TF prefix when several robots share one /tf
#                  (robots with --ma-frames, and every multi-robot simulation): the agent name
#
# Examples:
#   ./run_rviz.sh agent_2                  one real robot
#   ./run_rviz.sh agent_2 agent_2          a robot started with --ma-frames
#   ROS_DOMAIN_ID=31 ./run_rviz.sh sim_1 sim_1     a car in the simulation
if [[ "$1" == -h || "$1" == --help ]]; then awk 'NR>1{if(!/^#/)exit; sub(/^# ?/,""); print}' "$0"; exit 0; fi
AGENT="${1:-$AGENT_NAME}"
PREFIX="${2:-}"
if [ -z "$AGENT" ]; then echo "usage: $0 AGENT [FRAME_PREFIX]"; exit 1; fi
HERE="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
CFG="$(mktemp -d)/rviz_${AGENT}.rviz"
if [ -n "$PREFIX" ]; then
    sed -e "s#__AGENT__#$AGENT#g" -e "s#Fixed Frame: map_nav#Fixed Frame: $PREFIX/map_nav#" \
        -e "s#^        map_nav:#        $PREFIX/map_nav:#" -e "s#^        base_footprint:#        $PREFIX/base_footprint:#" \
        "$HERE/robot_view.rviz" > "$CFG"
else
    sed "s#__AGENT__#$AGENT#g" "$HERE/robot_view.rviz" > "$CFG"
fi
echo "RViz for agent '$AGENT' (fixed frame ${PREFIX:+$PREFIX/}map_nav; the map appears once the floor is calibrated)"
rviz2 -d "$CFG"
rm -rf "$(dirname "$CFG")"

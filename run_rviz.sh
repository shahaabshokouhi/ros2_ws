#!/bin/bash
# RViz view of one robot: occupancy grid, pose, trajectory, Nav2 plan and
# costmaps (goals: the "2D Goal Pose" button), and (with
# run_slam3.sh --teleop on the robot) its grayscale camera view.
#
#   ./run_rviz.sh [AGENT [FRAME_PREFIX]]   # default: $AGENT_NAME, no prefix
#
# FRAME_PREFIX: the robot's TF frame prefix when several robots share one /tf
# (multi-robot simulation, run_ma_sim_nav.sh: the agent name). The window is
# titled after the agent.
# Needs the same ROS_DOMAIN_ID as the robot.
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

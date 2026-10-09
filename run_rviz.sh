#!/bin/bash
# RViz view of one robot: occupancy grid, pose, trajectory and (with
# run_slam3.sh --teleop on the robot) its grayscale camera view.
#
#   ./run_rviz.sh [AGENT]        # default: $AGENT_NAME
#
# Needs the same ROS_DOMAIN_ID as the robot.
AGENT="${1:-$AGENT_NAME}"
if [ -z "$AGENT" ]; then echo "usage: $0 AGENT"; exit 1; fi
HERE="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
CFG="$(mktemp --suffix=.rviz)"
sed "s#__AGENT__#$AGENT#g" "$HERE/robot_view.rviz" > "$CFG"
echo "RViz for agent '$AGENT' (fixed frame map_nav; the map appears once the floor is calibrated)"
rviz2 -d "$CFG"
rm -f "$CFG"

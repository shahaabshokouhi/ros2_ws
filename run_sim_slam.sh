#!/bin/bash
# The robot's stack (ORB-SLAM3, grid, Nav2) on the Isaac Sim JetRacer.
# Start ./run_isaac_sim.sh first.
#
#   ./run_sim_slam.sh [--nav | --nav-wheels] [--teleop] [--no-grid] [--agent NAME]
#
#   --nav         Nav2, position feedback from SLAM (as on the robot)
#   --nav-wheels  Nav2 on "wheel odometry" (in the sim: the true pose) corrected by SLAM
#   --teleop      gray camera view for ./run_teleop.sh
# Same ROS domain as run_isaac_sim.sh ($SIM_DOMAIN, default 31).
HERE="$(cd "$(dirname "$0")" && pwd)"
AGENT=sim; GRID=true; NAV=false; NAV_POSE=slam; TELEOP=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --nav) NAV=true; shift ;;
        --nav-wheels) NAV=true; NAV_POSE=wheels; shift ;;
        --teleop) TELEOP=true; shift ;;
        --no-grid) GRID=false; shift ;;
        --agent) AGENT="$2"; shift 2 ;;
        *) echo "usage: $0 [--nav|--nav-wheels] [--teleop] [--no-grid] [--agent NAME]"; exit 1 ;;
    esac
done
source /opt/ros/humble/setup.bash
cd "$HERE" && colcon build --packages-select orb_slam3 jetracer_sim jetracer >/dev/null || exit 1
source "$HERE/install/setup.bash"
export ROS_DOMAIN_ID=${SIM_DOMAIN:-31}
echo "Simulated robot '$AGENT' on ROS_DOMAIN_ID=$ROS_DOMAIN_ID (grid $GRID, nav $NAV/$NAV_POSE, teleop $TELEOP)"
echo "RViz:   ROS_DOMAIN_ID=$ROS_DOMAIN_ID ./run_rviz.sh $AGENT"
echo "Drive:  ROS_DOMAIN_ID=$ROS_DOMAIN_ID ./run_teleop.sh $AGENT"
if [ "$NAV" = "true" ]; then
    echo "Nav2 starts once the floor is calibrated (a few seconds; until then it prints"
    echo "\"Timed out waiting for transform ... odom\"). Unseen space counts as free."
fi
exec ros2 launch jetracer_sim sim.launch.py agent:="$AGENT" grid:="$GRID" nav:="$NAV" \
    nav_pose:="$NAV_POSE" teleop:="$TELEOP"

#!/bin/bash
# PC (station). Drive ONE robot from this computer's keyboard and watch it in RViz (opens
# run_rviz.sh in the background; the keys are listed when it starts). Ctrl-C stops the
# robot and closes both. Robot side: ./run_slam.sh --teleop [--grid]
# (simulation: ./run_sim_slam.sh --teleop).
#
#   ./run_teleop.sh AGENT [PARAM:=VALUE ...]
#
#   AGENT          the robot's agent name
#   PARAM:=VALUE   keyboard node parameters: speed (default 0.15 m/s), turn (0.8 rad/s),
#                  max_linear (0.3), max_angular (1.5), hold_timeout (0.7 s), latch (false)
#
# Examples:
#   ./run_teleop.sh agent_2
#   ./run_teleop.sh agent_2 speed:=0.1
#   ROS_DOMAIN_ID=31 ./run_teleop.sh sim     the single simulated car
if [[ "$1" == -h || "$1" == --help ]]; then awk 'NR>1{if(!/^#/)exit; sub(/^# ?/,""); print}' "$0"; exit 0; fi
AGENT="$1"
if [ -z "$AGENT" ]; then echo "usage: $0 AGENT [param:=value ...]"; exit 1; fi
shift
HERE="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
if [ ! -x "$HERE/install/jetracer/lib/jetracer/teleop" ]; then
    (cd "$HERE" && colcon build --packages-select jetracer) || exit 1
fi
source "$HERE/install/setup.bash"
"$HERE/run_rviz.sh" "$AGENT" > /dev/null 2>&1 &
RVIZ=$!
PARAMS=(-p agent_name:="$AGENT")
for kv in "$@"; do PARAMS+=(-p "$kv"); done
ros2 run jetracer teleop --ros-args "${PARAMS[@]}"
kill $RVIZ 2>/dev/null

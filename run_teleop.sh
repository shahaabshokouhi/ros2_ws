#!/bin/bash
# Drive ONE robot from this computer's keyboard and watch it in RViz.
# Single-robot tests only. On the robot run:  ./run_slam3.sh --teleop [--grid]
#
#   ./run_teleop.sh AGENT [ros2 parameters, e.g. speed:=0.1]
#
# RViz opens in the background; this terminal reads the keyboard (keys are
# listed when it starts). Ctrl-C stops the robot and closes both.
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

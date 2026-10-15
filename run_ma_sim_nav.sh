#!/bin/bash
# SIMULATION, goals by hand. Several cars in Isaac Sim's room, each with its own SLAM and
# Nav2, and one RViz window per car: give goals with "2D Goal Pose" in that car's window.
# Everything is recorded; at the end the same report as run_ma_sim_test.sh is written.
# End the session with Enter (or Ctrl-C).
#
#   ./run_ma_sim_nav.sh [--spec FILE] [--method new|hq-mpshare] [--name NAME]
#                       [--headless] [--no-rviz] [--no-report]
#
#   --spec      robots and start poses (default src/jetracer_sim/config/ma_room_nav3.json:
#               sim_1, sim_2, sim_3 in the room; a spec needs at least two robots)
#   --method    how robots share maps (default new)
#   --name      label of the result folder
#   --headless  no Isaac window
#   --no-rviz   no RViz windows
#   --no-report do not write the report
#
# Results: ~/result/ma_nav/DATE_TIME_NAME_METHOD/.
#
# The room only (the lab: run_isaac_explore.sh).
#
# Examples:
#   ./run_ma_sim_nav.sh
#   ./run_ma_sim_nav.sh --method hq-mpshare --no-rviz
if [[ "$1" == -h || "$1" == --help ]]; then awk 'NR>1{if(!/^#/)exit; sub(/^# ?/,""); print}' "$0"; exit 0; fi
main() {
HERE="$(cd "$(dirname "$0")" && pwd)"
SPEC="$HERE/src/jetracer_sim/config/ma_room_nav3.json"
METHOD=new; NAME=""; HEADLESS=false; RVIZ=true; REPORT=true
while [[ $# -gt 0 ]]; do
    case "$1" in
        --spec) SPEC="$(realpath "$2")"; shift 2 ;;
        --method) METHOD="$2"; shift 2 ;;
        --name) NAME="$2"; shift 2 ;;
        --headless) HEADLESS=true; shift ;;
        --no-rviz) RVIZ=false; shift ;;
        --no-report) REPORT=false; shift ;;
        *) sed -n 2,25p "$0"; return 1 ;;
    esac
done
[ -f "$SPEC" ] || { echo "no spec $SPEC"; return 1; }
SPEC_NAME=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['name'])" "$SPEC")
AGENTS=$(python3 -c "import json,sys; print(' '.join(json.load(open(sys.argv[1]))['agents']))" "$SPEC")
N=$(echo $AGENTS | wc -w)
PREFIX=${AGENTS%%_1*}
[ "$AGENTS" = "$(for k in $(seq 1 $N); do printf '%s_%d ' "$PREFIX" $k; done | sed 's/ $//')" ] ||
    { echo "spec agents must be named <prefix>_1 .. <prefix>_N (got: $AGENTS)"; return 1; }

# Every session gets its own new folder; earlier results are never overwritten.
RUN="${MA_NAV_RESULT_DIR:-$HOME/result/ma_nav}/$(date +%Y%m%d_%H%M%S)_${NAME:-$SPEC_NAME}_$METHOD"
k=2; base="$RUN"; while [ -e "$RUN" ]; do RUN="${base}_$k"; k=$((k + 1)); done
RAW="$RUN/raw"
mkdir -p "$RAW/gt" "$RAW/online"
cp "$SPEC" "$RAW/spec.json"
cp "$HERE/src/jetracer_sim/config/room_obstacles.json" "$RAW/" 2>/dev/null
echo "Multi-robot nav session: $N robots ($AGENTS), method $METHOD -> $RUN"

stop_launches() {
    for p in $(pgrep -f "ros2 launch jetracer_sim"); do
        tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | grep -q "^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch" &&
            kill -INT -- -"$(ps -o pgid= $p | tr -d ' ')" 2>/dev/null
    done
}
stop_isaac() {
    for p in $(pgrep -x python); do
        tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | grep -q "jetracer_sim/isaac/run_isaac.py" && kill -INT $p
    done
}
stop_rviz() {   # only the windows this script opens (config rviz_<prefix>_<k>.rviz)
    for p in $(pgrep -x rviz2); do
        tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | grep -q "/rviz_${PREFIX}_[0-9]*\.rviz" && kill -INT $p
    done
}
slam_alive() {
    local n=0
    for p in $(pgrep -f "lib/orb_slam3/orb_slam3_node"); do
        tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | grep -q "lib/orb_slam3/orb_slam3_node" && n=$((n + 1))
    done
    echo $n
}
alive() {
    local n=0
    for p in $(pgrep -f "lib/orb_slam3/orb_slam3_node") $(pgrep -x python); do
        tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | grep -q "lib/orb_slam3/orb_slam3_node\|jetracer_sim/isaac/run_isaac.py" && n=$((n + 1))
    done
    echo $n
}
stop_launches; stop_isaac; stop_rviz
for i in $(seq 1 30); do [ "$(alive)" -eq 0 ] && break; sleep 2; done

cd "$HERE"
source /opt/ros/humble/setup.bash
colcon build --packages-select orb_slam3 jetracer_sim jetracer > "$RAW/build.log" 2>&1 || { echo "build failed: $RAW/build.log"; return 1; }
source install/setup.bash
export ROS_DOMAIN_ID=${SIM_DOMAIN:-31}
{
    echo "date: $(date -Iseconds)"; echo "mode: nav (goals by hand)"; echo "method: $METHOD"; echo "spec: $SPEC"
    echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    echo "ros2_ws: $(git -C "$HERE" rev-parse --short HEAD) $(git -C "$HERE" status --porcelain | wc -l) changed files"
    echo "ORB_SLAM3: $(git -C "$HOME/ORB_SLAM3" rev-parse --short HEAD) $(git -C "$HOME/ORB_SLAM3" status --porcelain | wc -l) changed files"
    env | grep '^MA_NEW_' || true
} > "$RAW/env.txt"

# 1. Isaac Sim with the cars at their start poses.
POSES=$(python3 src/jetracer_sim/jetracer_sim/routes.py "$SPEC")
ISAAC_ARGS=(--agent "$PREFIX" --agents "$N" --poses "$POSES" --gt-log "$RAW/gt")
$HEADLESS && ISAAC_ARGS+=(--headless)
(systemd-run --user --scope -p MemoryMax=10G setsid ./run_isaac_sim.sh "${ISAAC_ARGS[@]}" > "$RAW/isaac.log" 2>&1 &)
echo "starting Isaac Sim ..."
for i in $(seq 1 100); do sleep 3; grep -q "running \|Traceback" "$RAW/isaac.log" && break; done
grep -q "running " "$RAW/isaac.log" || { echo "Isaac did not start: $RAW/isaac.log"; stop_isaac; return 1; }
sleep 5

# 2. One SLAM + Nav2 stack per robot, in its own TF frames.
for a in $AGENTS; do
    mkdir -p "$RAW/$a/eval"
    (MA_NEW_EVAL_DIR="$RAW/$a/eval" systemd-run --user --scope -p MemoryMax=4G setsid \
        ros2 launch jetracer_sim sim.launch.py agent:="$a" ma_method:="$METHOD" nav:=true frame_prefix:="$a" \
        > "$RAW/$a/slam.log" 2>&1 &)
done
echo "starting $N SLAM + Nav2 stacks ..."
for i in $(seq 1 120); do
    sleep 2; c=0
    for a in $AGENTS; do grep -q "Managed nodes are active" "$RAW/$a/slam.log" && c=$((c + 1)); done
    [ $c -eq $N ] && break
done
echo "Nav2 active on $c of $N robots"

# 3. Record, and open one RViz per robot.
setsid ros2 run jetracer_sim route_driver --ros-args -p spec:="$SPEC" -p out:="$RAW/online" -p drive:=false \
    > "$RAW/recorder.log" 2>&1 &
REC=$!
date +%s.%N > "$RAW/drive_start.txt"
if $RVIZ; then
    for a in $AGENTS; do (setsid ./run_rviz.sh "$a" "$a" > "$RAW/rviz_$a.log" 2>&1 &); done
fi
echo
echo "Ready. Give goals in each robot's RViz window (title rviz_<robot>.rviz):"
echo "  click \"2D Goal Pose\", then click the target on the map and drag towards the final heading."
echo "  Each robot's map starts at its own start pose, facing +x (red axis)."
echo "Press Enter here (or Ctrl-C) to end the session and write the report."
trap 'true' INT
read -r _ || true
trap '' INT                    # from here on, finish cleanly whatever is pressed
date +%s.%N > "$RAW/drive_end.txt"

# 4. Stop: robots save their maps (final optimisation), then the rest.
echo "ending: cancelling goals, saving maps ..."
for a in $AGENTS; do
    ros2 service call "/$a/navigate_to_pose/_action/cancel_goal" action_msgs/srv/CancelGoal "{}" > /dev/null 2>&1 &
done
sleep 2
kill -INT -- -"$REC" 2>/dev/null
stop_launches
for i in $(seq 1 150); do [ "$(slam_alive)" -eq 0 ] && break; sleep 2; done
stop_rviz; stop_isaac
for i in $(seq 1 30); do [ "$(alive)" -eq 0 ] && break; sleep 2; done
for a in $AGENTS; do
    echo "  $a: $(grep -a -o 'wrote [0-9]* keyframes' "$RAW/$a/slam.log" | tail -1), $(grep -a -o 'wrote [0-9]* map points' "$RAW/$a/slam.log" | tail -1), $(($(wc -l < "$RAW/online/${a}_goals.csv" 2>/dev/null || echo 1) - 1)) goals"
done

# 5. Report.
if $REPORT; then
    echo "scoring and writing the report ..."
    ~/venvs/env_isaaclab/bin/python src/jetracer_sim/eval/ma_report.py "$RUN" > "$RAW/report.log" 2>&1 ||
        { echo "report failed: $RAW/report.log"; tail -5 "$RAW/report.log"; return 1; }
    google-chrome --headless=new --no-pdf-header-footer --print-to-pdf="$RUN/report.pdf" "$RUN/report.html" > /dev/null 2>&1
    echo "report: $RUN/report.html (and report.pdf)"
fi
}
main "$@"
exit $?

#!/bin/bash
# Multi-robot test in Isaac Sim: several JetRacers in one simulated room, each
# running its own SLAM stack (multi-robot method new or hq-mpshare), driven
# repeatably along a route; then scored against the simulator's ground truth.
#
#   ./run_ma_sim_test.sh [--spec FILE] [--method new|hq-mpshare] [--name NAME]
#                        [--gui] [--settle SECONDS] [--no-report]
#
#   --spec     test spec (default src/jetracer_sim/config/ma_room_loop3.json:
#              three cars on one loop in the furnished room, a third of a lap apart)
#   --method   multi-robot method for every robot (default new)
#   --name     label for the result folder (default: the spec's name)
#   --gui      show the Isaac window (default headless)
#   --settle   seconds to let sharing finish after driving (default 20)
#
# Results: ~/result/ma_sim/<date>_<name>_<method>/ (or $MA_RESULT_DIR/...):
#   report.html, report.pdf, metrics.json, figures/, tables/  the report
#   raw/   spec, ground truth (raw/gt/<agent>_gt.csv), each robot's SLAM
#          evaluation files (raw/<agent>/eval/: keyframes, map points,
#          alignments, verifications, timing), live SLAM poses (raw/online/),
#          all logs, and the software versions (raw/env.txt)
# Re-score a run later with:
#   ~/venvs/env_isaaclab/bin/python src/jetracer_sim/eval/ma_report.py <run folder>
#
# Uses ROS_DOMAIN_ID 31 ($SIM_DOMAIN). Stops any simulation from this
# workspace that is still running. Takes about 5 minutes for the default spec.
# The whole script is one function, read completely before it runs: editing
# or pulling this file during a test cannot change the running test.
main() {
HERE="$(cd "$(dirname "$0")" && pwd)"
SPEC="$HERE/src/jetracer_sim/config/ma_room_loop3.json"
METHOD=new; NAME=""; GUI=false; SETTLE=20; REPORT=true
while [[ $# -gt 0 ]]; do
    case "$1" in
        --spec) SPEC="$(realpath "$2")"; shift 2 ;;
        --method) METHOD="$2"; shift 2 ;;
        --name) NAME="$2"; shift 2 ;;
        --gui) GUI=true; shift ;;
        --settle) SETTLE="$2"; shift 2 ;;
        --no-report) REPORT=false; shift ;;
        *) sed -n 2,26p "$0"; return 1 ;;
    esac
done
[ -f "$SPEC" ] || { echo "no spec $SPEC"; exit 1; }
SPEC_NAME=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['name'])" "$SPEC")
AGENTS=$(python3 -c "import json,sys; print(' '.join(json.load(open(sys.argv[1]))['agents']))" "$SPEC")
N=$(echo $AGENTS | wc -w)
PREFIX=${AGENTS%%_1 *}
[ "$AGENTS" = "$(for k in $(seq 1 $N); do printf '%s_%d ' "$PREFIX" $k; done | sed 's/ $//')" ] ||
    { echo "spec agents must be named <prefix>_1 .. <prefix>_N (got: $AGENTS)"; exit 1; }

# Every test gets its own new folder; earlier results are never overwritten.
RUN="${MA_RESULT_DIR:-$HOME/result/ma_sim}/$(date +%Y%m%d_%H%M%S)_${NAME:-$SPEC_NAME}_$METHOD"
k=2; base="$RUN"; while [ -e "$RUN" ]; do RUN="${base}_$k"; k=$((k + 1)); done
RAW="$RUN/raw"
mkdir -p "$RAW/gt" "$RAW/online"
cp "$SPEC" "$RAW/spec.json"
cp "$HERE/src/jetracer_sim/config/room_obstacles.json" "$RAW/" 2>/dev/null
echo "Multi-robot sim test: $N robots ($AGENTS), method $METHOD -> $RUN"

# Stop what is left of earlier simulations from this workspace (by process
# group for launches; never by a pattern that would match this shell).
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
alive() {   # number of SLAM nodes or Isaac processes still running
    local n=0
    for p in $(pgrep -f "lib/orb_slam3/orb_slam3_node") $(pgrep -x python); do
        tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | grep -q "lib/orb_slam3/orb_slam3_node\|jetracer_sim/isaac/run_isaac.py" && n=$((n + 1))
    done
    echo $n
}
stop_launches; stop_isaac
for i in $(seq 1 30); do [ "$(alive)" -eq 0 ] && break; sleep 2; done

cd "$HERE"
source /opt/ros/humble/setup.bash
colcon build --packages-select orb_slam3 jetracer_sim jetracer > "$RAW/build.log" 2>&1 || { echo "build failed: $RAW/build.log"; exit 1; }
source install/setup.bash
export ROS_DOMAIN_ID=${SIM_DOMAIN:-31}
{
    echo "date: $(date -Iseconds)"; echo "method: $METHOD"; echo "spec: $SPEC"; echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    echo "ros2_ws: $(git -C "$HERE" rev-parse --short HEAD) $(git -C "$HERE" status --porcelain | wc -l) changed files"
    echo "ORB_SLAM3: $(git -C "$HOME/ORB_SLAM3" rev-parse --short HEAD) $(git -C "$HOME/ORB_SLAM3" status --porcelain | wc -l) changed files"
    env | grep '^MA_NEW_' || true
} > "$RAW/env.txt"

# 1. Isaac Sim with N cars at their start poses on the route.
POSES=$(python3 src/jetracer_sim/jetracer_sim/routes.py "$SPEC")
ISAAC_ARGS=(--agent "$PREFIX" --agents "$N" --poses "$POSES" --gt-log "$RAW/gt")
$GUI || ISAAC_ARGS+=(--headless)
(systemd-run --user --scope -p MemoryMax=10G setsid ./run_isaac_sim.sh "${ISAAC_ARGS[@]}" > "$RAW/isaac.log" 2>&1 &)
echo "starting Isaac Sim ..."
for i in $(seq 1 100); do sleep 3; grep -q "running \|Traceback" "$RAW/isaac.log" && break; done
grep -q "running " "$RAW/isaac.log" || { echo "Isaac did not start: $RAW/isaac.log"; stop_isaac; exit 1; }
sleep 5

# 2. One SLAM stack per robot, writing its evaluation files.
for a in $AGENTS; do
    mkdir -p "$RAW/$a/eval"
    (MA_NEW_EVAL_DIR="$RAW/$a/eval" systemd-run --user --scope -p MemoryMax=4G setsid \
        ros2 launch jetracer_sim sim.launch.py agent:="$a" ma_method:="$METHOD" > "$RAW/$a/slam.log" 2>&1 &)
done
echo "starting $N SLAM stacks ..."
for i in $(seq 1 90); do
    sleep 2; c=0
    for a in $AGENTS; do grep -q "camera mount fitted" "$RAW/$a/slam.log" && c=$((c + 1)); done
    [ $c -eq $N ] && break
done
echo "floor calibrated on $c of $N robots"
sleep 3

# 3. Drive the route (blocks until every car has finished).
echo "driving the route ..."
date +%s.%N > "$RAW/drive_start.txt"
ros2 run jetracer_sim route_driver --ros-args -p spec:="$SPEC" -p out:="$RAW/online" > "$RAW/driver.log" 2>&1
date +%s.%N > "$RAW/drive_end.txt"
grep -a "drove" "$RAW/driver.log" | sed 's/.*\]: /  /'

# 4. Let the robots finish sharing, then stop SLAM gracefully: on Ctrl-C each
# node runs its final optimisation and writes its keyframes and map points.
echo "letting sharing settle for $SETTLE s ..."
sleep "$SETTLE"
echo "stopping SLAM (final optimisation and export) ..."
stop_launches
for i in $(seq 1 150); do
    n=0; for p in $(pgrep -f "lib/orb_slam3/orb_slam3_node"); do
        tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | grep -q "lib/orb_slam3/orb_slam3_node" && n=$((n + 1)); done
    [ $n -eq 0 ] && break; sleep 2
done
stop_isaac
for i in $(seq 1 30); do [ "$(alive)" -eq 0 ] && break; sleep 2; done
for a in $AGENTS; do
    echo "  $a: $(grep -a -o 'wrote [0-9]* keyframes' "$RAW/$a/slam.log" | tail -1), $(grep -a -o 'wrote [0-9]* map points' "$RAW/$a/slam.log" | tail -1)"
done

# 5. Report.
if $REPORT; then
    echo "scoring and writing the report ..."
    ~/venvs/env_isaaclab/bin/python src/jetracer_sim/eval/ma_report.py "$RUN" > "$RAW/report.log" 2>&1 ||
        { echo "report failed: $RAW/report.log"; tail -5 "$RAW/report.log"; exit 1; }
    google-chrome --headless=new --no-pdf-header-footer --print-to-pdf="$RUN/report.pdf" "$RUN/report.html" > /dev/null 2>&1
    echo "report: $RUN/report.html (and report.pdf)"
fi
}
main "$@"
exit $?

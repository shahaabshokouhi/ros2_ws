#!/bin/bash
# SIMULATION, exploration. Three cars in Isaac Sim, each with its own SLAM and Nav2, each
# choosing its own goals with an ma_explore planner (the trained policy or a baseline);
# then the SLAM report (paths vs ground truth, merging) and the exploration summary
# (coverage over time, decisions). Needs the ma_explore repository and its venv
# ($MA_EXPLORE_DIR, default ~/ma_explore). The training's Isaac evaluations and the
# stored baselines (python -m ma_explore.baseline_bank) run this script.
#
#   ./run_isaac_explore.sh [--planner PLANNER] [--checkpoint FILE] [--env big|room] [--spec FILE]
#                          [--method new|hq-mpshare] [--time-limit S] [--stop-coverage F]
#                          [--name NAME] [--device cpu|cuda] [--headless] [--view CAMERA]
#                          [--no-report] [--planner-param NAME:=VALUE ...]
#
#   --planner     rl (default: the trained policy in --checkpoint); rules: coordinated, nearest,
#                 utility, random; published methods: mmpf, mtsp, burgard, alc
#   --checkpoint  the policy (default: best.pt of the newest run in ~/result/ma_explore/runs)
#   --env         big (default): the 22 x 16 m lab (~/isaac_worlds/big_env, spec
#                 config/ma_big_nav3.json, 1200 s); room: the 6 x 6 m room (ma_room_nav3.json, 300 s)
#   --spec        robots, start poses, world and mission length (instead of --env)
#   --method      how robots share maps (default new)
#   --time-limit  mission length in seconds (default: the spec's time_limit)
#   --stop-coverage F  end early once the team has covered F of the world (default: never)
#   --name        label of the result folder
#   --device      where the policy runs (default cpu)
#   --headless    no windows (default: the Isaac window plus a live 2D view of coverage,
#                 paths and goals; closing them does not stop the session)
#   --view        Isaac camera: Top (default), Corner_NE, Corner_NW, Corner_SW, Corner_SE
#   --no-report   do not write the reports
#   --planner-param  an extra planner parameter for every robot, e.g. prefer_revisit:=true
#
# Results: ~/result/ma_explore/isaac/DATE_TIME_NAME_PLANNER/. Enter (or Ctrl-C) ends a session
# early. Compare sessions: python -m ma_explore.isaac_report DIR1 DIR2 ... (in ~/ma_explore).
# Uses ROS_DOMAIN_ID 31 ($SIM_DOMAIN). Needs the ros2_ws and ORB_SLAM3 builds.
#
# Examples:
#   ./run_isaac_explore.sh                                  the newest policy in the lab
#   ./run_isaac_explore.sh --checkpoint ~/result/ma_explore/runs/RUN/checkpoints/best.pt --headless
#   ./run_isaac_explore.sh --planner mmpf --env room        a published method in the room
if [[ "$1" == -h || "$1" == --help ]]; then awk 'NR>1{if(!/^#/)exit; sub(/^# ?/,""); print}' "$0"; exit 0; fi
main() {
MAX="${MA_EXPLORE_DIR:-$HOME/ma_explore}"
WS="$(cd "$(dirname "$0")" && pwd)"
PY="$HOME/venvs/ma_explore/bin/python"
SPEC=""; ENV_NAME=big
PPARAMS=(); PLANNER=rl; CKPT=""; METHOD=new; NAME=""; LIMIT=""; STOPCOV=1.01; DEVICE=cpu; HEADLESS=false; VIEW=Top; REPORT=true
while [[ $# -gt 0 ]]; do
    case "$1" in
        --planner) PLANNER="$2"; shift 2 ;;
        --checkpoint) CKPT="$(realpath "$2")"; shift 2 ;;
        --spec) SPEC="$(realpath "$2")"; shift 2 ;;
        --env) ENV_NAME="$2"; shift 2 ;;
        --method) METHOD="$2"; shift 2 ;;
        --time-limit) LIMIT="$2"; shift 2 ;;
        --stop-coverage) STOPCOV="$2"; shift 2 ;;
        --name) NAME="$2"; shift 2 ;;
        --device) DEVICE="$2"; shift 2 ;;
        --headless) HEADLESS=true; shift ;;
        --view) VIEW="$2"; shift 2 ;;
        --planner-param) PPARAMS+=(-p "$2"); shift 2 ;;
        --no-report) REPORT=false; shift ;;
        *) sed -n 2,40p "$0"; return 1 ;;
    esac
done
MODULE=ma_explore.ros_planner; MPARAMS=()
case "$PLANNER" in      # published methods: the shared robot stack's own policy slot holds a rule it does not use
    mmpf|mtsp|burgard|alc) MODULE=ma_explore.classic_planner; MPARAMS=(-p method:="$PLANNER" -p policy:=coordinated) ;;
esac
SPEC_PARAMS=()          # the spec's planner_params (e.g. the lab: alc_scale 1.0), set below once the spec is known
if [ "$PLANNER" = rl ]; then
    [ -n "$CKPT" ] || CKPT="$(ls -td "$HOME"/result/ma_explore/runs/*/ 2>/dev/null | head -1)checkpoints/best.pt"
    [ -f "$CKPT" ] || { echo "no checkpoint $CKPT"; return 1; }
    POLICY="rl:$CKPT"
else
    POLICY="$PLANNER"
fi
[ -n "$SPEC" ] || SPEC="$WS/src/jetracer_sim/config/ma_${ENV_NAME}_nav3.json"
[ -f "$SPEC" ] || { echo "no spec $SPEC"; return 1; }
spec_get() { python3 -c "import json,os,sys; v=json.load(open(sys.argv[1])).get(sys.argv[2]); print(os.path.expanduser(str(v)) if v is not None else '')" "$SPEC" "$1"; }
WORLD=$(spec_get world)
[ -n "$LIMIT" ] || LIMIT=$(spec_get time_limit)
[ -n "$LIMIT" ] || LIMIT=300
if [ "$MODULE" = ma_explore.classic_planner ]; then
    for kv in $(python3 -c "import json,sys; [print(f'{k}:={v}') for k, v in json.load(open(sys.argv[1])).get('planner_params', {}).items()]" "$SPEC"); do
        SPEC_PARAMS+=(-p "$kv")
    done
fi
[ -z "$WORLD" ] || [ -f "$WORLD/free_grid.json" ] || { echo "no world at $WORLD (spec $SPEC)"; return 1; }
SPEC_NAME=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['name'])" "$SPEC")
AGENTS=$(python3 -c "import json,sys; print(' '.join(json.load(open(sys.argv[1]))['agents']))" "$SPEC")
N=$(echo $AGENTS | wc -w)
PREFIX=${AGENTS%%_1 *}

RUN="${MA_EXPLORE_ISAAC_DIR:-$HOME/result/ma_explore/isaac}/$(date +%Y%m%d_%H%M%S)_${NAME:-$SPEC_NAME}_$PLANNER"
k=2; base="$RUN"; while [ -e "$RUN" ]; do RUN="${base}_$k"; k=$((k + 1)); done
RAW="$RUN/raw"
mkdir -p "$RAW/gt" "$RAW/online" "$RAW/planner"
cp "$SPEC" "$RAW/spec.json"
if [ -n "$WORLD" ]; then            # the world's ground truth, kept with the session (reports, scoring)
    mkdir -p "$RAW/world"; cp "$WORLD/free_grid.json" "$WORLD/obstacles.json" "$RAW/world/"
else
    cp "$WS/src/jetracer_sim/config/room_obstacles.json" "$RAW/" 2>/dev/null
fi
echo "Exploration session: $N robots ($AGENTS) in ${WORLD:-the room}, planner $POLICY, SLAM method $METHOD, $LIMIT s -> $RUN"

stop_launches() {
    for p in $(pgrep -f "ros2 launch jetracer_sim"); do
        tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | grep -q "^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch" &&
            kill -INT -- -"$(ps -o pgid= $p | tr -d ' ')" 2>/dev/null
    done
}
stop_nav2_leftovers() {   # Nav2 nodes of these robots that outlived their launch (14 October: a lifecycle
    # manager left over from an earlier session ignored SIGINT and, under the same name, kept the next
    # session's Nav2 of that robot from ever becoming active)
    local sig ps_=""
    for p in $(pgrep -f "/opt/ros/humble/lib/nav2_"); do
        local cmd; cmd=$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)
        [[ "$cmd" == /opt/ros/humble/lib/nav2_* ]] || continue
        for a in $AGENTS; do [[ "$cmd" == *"__ns:=/$a "* ]] && ps_="$ps_ $p"; done
    done
    [ -z "$ps_" ] && return
    echo "stopping Nav2 processes left over from an earlier session:$ps_"
    for sig in TERM KILL; do
        for p in $ps_; do kill -$sig $p 2>/dev/null; done
        sleep 3
    done
}
stop_isaac() {
    for p in $(pgrep -x python); do
        tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | grep -q "jetracer_sim/isaac/run_isaac.py" && kill -INT $p
    done
}
stop_planners() {   # planners, the monitor, the live view and the recorder of this script
    for p in $(pgrep -f "ma_explore[.](ros_planner|classic_planner|isaac_monitor|isaac_view)|jetracer_sim/route_driver"); do
        tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | grep -q "ma_explore.ros_planner\|ma_explore.classic_planner\|ma_explore.isaac_monitor\|ma_explore.isaac_view\|route_driver" &&
            kill -INT $p 2>/dev/null
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
stop_planners; stop_launches; stop_isaac
for i in $(seq 1 30); do [ "$(alive)" -eq 0 ] && break; sleep 2; done
stop_nav2_leftovers

cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --packages-select orb_slam3 jetracer_sim jetracer > "$RAW/build.log" 2>&1 || { echo "build failed: $RAW/build.log"; return 1; }
source install/setup.bash
export ROS_DOMAIN_ID=${SIM_DOMAIN:-31}
{
    echo "date: $(date -Iseconds)"; echo "mode: explore (planner $PLANNER)"; echo "method: $METHOD"; echo "spec: $SPEC"
    echo "planner: $POLICY"; echo "planner_params: ${PPARAMS[*]}"; echo "time_limit: $LIMIT"; echo "stop_coverage: $STOPCOV"; echo "device: $DEVICE"
    echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    echo "ros2_ws: $(git -C "$WS" rev-parse --short HEAD) $(git -C "$WS" status --porcelain | wc -l) changed files"
    echo "ORB_SLAM3: $(git -C "$HOME/ORB_SLAM3" rev-parse --short HEAD) $(git -C "$HOME/ORB_SLAM3" status --porcelain | wc -l) changed files"
    echo "ma_explore: $(git -C "$MAX" rev-parse --short HEAD) $(git -C "$MAX" status --porcelain | wc -l) changed files"
    env | grep '^MA_NEW_' || true
} > "$RAW/env.txt"

# 1. Isaac Sim with the cars at their start poses.
POSES=$(python3 src/jetracer_sim/jetracer_sim/routes.py "$SPEC")
ISAAC_ARGS=(--agent "$PREFIX" --agents "$N" --poses="$POSES" --gt-log "$RAW/gt")   # "=": a pose may start with "-"
[ -n "$WORLD" ] && ISAAC_ARGS+=(--world "$WORLD")
if $HEADLESS; then ISAAC_ARGS+=(--headless); else ISAAC_ARGS+=(--view "$VIEW"); fi
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

# 3. Recorder (paths for the SLAM report), coverage monitor, one planner per robot.
setsid ros2 run jetracer_sim route_driver --ros-args -p spec:="$SPEC" -p out:="$RAW/online" -p drive:=false \
    > "$RAW/recorder.log" 2>&1 &
START="$RAW/planner/start"
(cd "$MAX" && setsid "$PY" -m ma_explore.isaac_monitor --ros-args -p spec:="$SPEC" -p out:="$RAW/planner" \
    -p stop_coverage:="$STOPCOV" -p time_limit:="$LIMIT" -p start_file:="$START" > "$RAW/planner/monitor.log" 2>&1 &)
for a in $AGENTS; do
    (cd "$MAX" && systemd-run --user --scope -q -p MemoryMax=3G setsid "$PY" -m $MODULE --ros-args -p policy:="$POLICY" "${MPARAMS[@]}" \
        -p agent:="$a" -p agents:="$AGENTS" -p device:="$DEVICE" \
        -p out:="$RAW/planner" -p start_file:="$START" -p mission_s:="$LIMIT" "${SPEC_PARAMS[@]}" "${PPARAMS[@]}" > "$RAW/planner/$a.log" 2>&1 &)
done
$HEADLESS || (cd "$MAX" && setsid "$PY" -m ma_explore.isaac_view --ros-args -p spec:="$SPEC" -p gt_dir:="$RAW/gt" \
    -p start_file:="$START" > "$RAW/planner/view.log" 2>&1 &)
echo "waiting for the planners (policy warm-up, first map, Nav2) ..."
for i in $(seq 1 120); do
    sleep 2; c=0
    for a in $AGENTS; do [ -e "$RAW/planner/$a.ready" ] && c=$((c + 1)); done
    [ $c -eq $N ] && break
done
[ $c -eq $N ] || echo "only $c of $N planners ready after 4 min; starting anyway (see $RAW/planner/*.log)"
date +%s.%N > "$RAW/drive_start.txt"
touch "$START"
echo "exploring: $(python3 -c "print('a mission of $LIMIT s' if $STOPCOV > 1 else 'until ' + str(round(100*$STOPCOV)) + ' % team coverage or $LIMIT s')") (Enter or Ctrl-C ends it early)"

STOP=false
trap 'STOP=true' INT
while [ ! -e "$RAW/planner/done.txt" ] && ! $STOP; do
    if [ -t 0 ]; then read -r -t 5 _ && break; else sleep 5; fi      # Enter ends early
    last=$(tail -1 "$RAW/planner/coverage.csv" 2>/dev/null | cut -d, -f2,3)
    [ -n "$last" ] && printf '\r  %s s  team coverage %s   ' "${last%%,*}" "${last##*,}"
done
trap '' INT                    # from here on, finish cleanly whatever is pressed
echo
cat "$RAW/planner/done.txt" 2>/dev/null | head -1 || echo "ended by hand"
date +%s.%N > "$RAW/drive_end.txt"

# 4. Stop: planners cancel their goals, robots save their maps, then the rest.
echo "ending: stopping planners, saving maps ..."
stop_planners
sleep 2
stop_launches
for i in $(seq 1 150); do [ "$(slam_alive)" -eq 0 ] && break; sleep 2; done
stop_isaac
for i in $(seq 1 30); do [ "$(alive)" -eq 0 ] && break; sleep 2; done
sleep 3; stop_nav2_leftovers
for a in $AGENTS; do
    echo "  $a: $(grep -a -o 'wrote [0-9]* keyframes' "$RAW/$a/slam.log" | tail -1), $(($(wc -l < "$RAW/planner/${a}_decisions.csv" 2>/dev/null || echo 1) - 1)) decisions"
done

# 5. Reports.
if $REPORT; then
    echo "scoring and writing the reports ..."
    ~/venvs/env_isaaclab/bin/python src/jetracer_sim/eval/ma_report.py "$RUN" > "$RAW/report.log" 2>&1 ||
        echo "SLAM report failed: $RAW/report.log"
    [ -f "$RUN/report.html" ] && google-chrome --headless=new --no-pdf-header-footer \
        --print-to-pdf="$RUN/report.pdf" "$RUN/report.html" > /dev/null 2>&1
    (cd "$MAX" && "$PY" -m ma_explore.isaac_report "$RUN" > "$RAW/explore_report.log" 2>&1) ||
        echo "exploration summary failed: $RAW/explore_report.log"
    echo "reports: $RUN/report.html (SLAM), $RUN/explore.html (exploration)"
fi
}
# One compound command, parsed whole before it runs: bash never reads past it, so editing this file
# during a session cannot change or break the running one (15 Oct 2026: it read the shifted tail).
{ main "$@"; exit $?; }

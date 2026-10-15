#!/bin/bash
# PC (station), real robots. Exploration with the planners on this PC: an ma_explore planner
# (the trained policy or a baseline) chooses each robot's next goal and sends it to the
# robot's Nav2, with a live view of what each robot knows and decides. Needs the ma_explore
# repository and its venv on this PC ($MA_EXPLORE_DIR, default ~/ma_explore) and the
# robots' ROS_DOMAIN_ID. (Alternative without a PC: run_slam.sh --explore on each robot.)
# Robot side, on each robot:
#   ./run_slam.sh --nav                one robot
#   ./run_slam.sh --nav --ma-frames    several robots on one ROS domain
#
#   ./run_robot_explore.sh AGENT [AGENT ...] [--planner rl|coordinated|nearest|utility]
#                          [--checkpoint FILE] [--frames plain|prefixed] [--device cpu|cuda]
#                          [--time-limit S] [--name NAME] [--no-view] [--rviz] [--record]
#                          [--planner-param NAME:=VALUE ...]
#
#   AGENT          each robot's AGENT_NAME
#   --planner      rl (default: the trained policy) or a rule baseline
#   --checkpoint   the policy (default: best.pt of the newest run in ~/result/ma_explore/runs)
#   --frames       plain: 'map_nav' (robots with --nav); prefixed: 'AGENT/map_nav' (--ma-frames).
#                  Default: plain for one robot, prefixed for several
#   --device       where the policy runs (default cpu)
#   --time-limit   stop after S seconds (default: until Enter or Ctrl-C)
#   --name         label of the result folder
#   --no-view      no live window (ma_explore.robot_view)
#   --rviz         also open RViz per robot (run_rviz.sh)
#   --record       also record a rosbag of the maps, poses and decisions
#   --planner-param  an extra planner parameter for every robot, e.g. prefer_revisit:=true
#
# Results: ~/result/ma_explore/robot/DATE_TIME_NAME/ (planner logs, AGENT_decisions.csv,
# summary.txt). Enter or Ctrl-C ends it: the planners cancel their goals and the cars stop.
# Keep a hand on the robot's stop on the first runs.
#
# Examples:
#   ./run_robot_explore.sh agent_2
#   ./run_robot_explore.sh agent_1 agent_2 agent_3 --time-limit 600 --rviz
#   ./run_robot_explore.sh agent_2 --planner coordinated
if [[ "$1" == -h || "$1" == --help ]]; then awk 'NR>1{if(!/^#/)exit; sub(/^# ?/,""); print}' "$0"; exit 0; fi
main() {
MAX="${MA_EXPLORE_DIR:-$HOME/ma_explore}"
WS="$(cd "$(dirname "$0")" && pwd)"
PY="$HOME/venvs/ma_explore/bin/python"
AGENTS=(); PPARAMS=(); PLANNER=rl; CKPT=""; FRAMES=""; DEVICE=cpu; LIMIT=0; NAME=""
VIEW=true; RVIZ=false; RECORD=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --planner) PLANNER="$2"; shift 2 ;;
        --checkpoint) CKPT="$(realpath "$2")"; shift 2 ;;
        --frames) FRAMES="$2"; shift 2 ;;
        --device) DEVICE="$2"; shift 2 ;;
        --time-limit) LIMIT="$2"; shift 2 ;;
        --name) NAME="$2"; shift 2 ;;
        --no-view) VIEW=false; shift ;;
        --rviz) RVIZ=true; shift ;;
        --record) RECORD=true; shift ;;
        --planner-param) PPARAMS+=(-p "$2"); shift 2 ;;
        -*) sed -n 2,34p "$0"; return 1 ;;
        *) AGENTS+=("$1"); shift ;;
    esac
done
[ ${#AGENTS[@]} -gt 0 ] || { sed -n 2,34p "$0"; return 1; }
N=${#AGENTS[@]}
[ -n "$FRAMES" ] || { [ "$N" -eq 1 ] && FRAMES=plain || FRAMES=prefixed; }
case "$FRAMES" in plain) FP=none ;; prefixed) FP=agent ;; *) echo "--frames plain|prefixed"; return 1 ;; esac
if [ "$PLANNER" = rl ]; then
    [ -n "$CKPT" ] || CKPT="$(ls -td "$HOME"/result/ma_explore/runs/*/ 2>/dev/null | head -1)checkpoints/best.pt"
    [ -f "$CKPT" ] || { echo "no checkpoint $CKPT"; return 1; }
    POLICY="rl:$CKPT"
else
    POLICY="$PLANNER"
fi
source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
RUN="$HOME/result/ma_explore/robot/$(date +%Y%m%d_%H%M%S)_${NAME:-$(IFS=_; echo "${AGENTS[*]}")}_$PLANNER"
mkdir -p "$RUN"
{
    echo "date: $(date -Iseconds)"; echo "agents: ${AGENTS[*]}"; echo "planner: $POLICY"; echo "frames: $FRAMES"
    echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"; echo "ma_explore: $(git -C "$MAX" rev-parse --short HEAD 2>/dev/null)"
} > "$RUN/env.txt"
echo "Exploration on ${AGENTS[*]} (planner $POLICY, frames $FRAMES, ROS_DOMAIN_ID ${ROS_DOMAIN_ID:-0}) -> $RUN"

# 1. Are the robots there? (their SLAM grid and Nav2)
for a in "${AGENTS[@]}"; do
    printf '  %s: ' "$a"
    for i in $(seq 1 30); do
        T=$(ros2 topic list 2>/dev/null; ros2 action list 2>/dev/null)
        echo "$T" | grep -qx "/$a/map" && echo "$T" | grep -qx "/$a/navigate_to_pose" && break
        sleep 2
    done
    if echo "$T" | grep -qx "/$a/map" && echo "$T" | grep -qx "/$a/navigate_to_pose"; then
        echo "grid and Nav2 found"
    else
        echo "NOT FOUND (/$a/map or /$a/navigate_to_pose): is ./run_slam.sh --nav running there, same ROS_DOMAIN_ID?"
        echo "  (the grid appears once the floor is calibrated: keep the floor in the camera's view)"
        return 1
    fi
done

# 2. Planners, the view, RViz, the recorder (each one's pid is kept to stop exactly these).
PIDS=()
TEAM="${AGENTS[*]}"
for a in "${AGENTS[@]}"; do
    cd "$MAX"
    setsid "$PY" -m ma_explore.ros_planner --ros-args -p agent:="$a" -p agents:="$TEAM" -p policy:="$POLICY" \
        -p device:="$DEVICE" -p out:="$RUN" -p frame_prefix:="$FP" "${PPARAMS[@]}" > "$RUN/$a.log" 2>&1 &
    PIDS+=($!)
done
if $VIEW; then
    cd "$MAX"; setsid "$PY" -m ma_explore.robot_view --ros-args -p agents:="$TEAM" > "$RUN/view.log" 2>&1 &
    PIDS+=($!)
fi
if $RVIZ; then
    for a in "${AGENTS[@]}"; do
        [ "$FRAMES" = prefixed ] && P="$a" || P=""
        (cd "$WS" && setsid ./run_rviz.sh "$a" $P > "$RUN/rviz_$a.log" 2>&1 &)
    done
fi
if $RECORD; then
    TOPICS=()
    for a in "${AGENTS[@]}"; do
        TOPICS+=("/$a/map" "/$a/orb_slam3/odom" "/$a/orb_slam3/slam_metrics" "/$a/orb_slam3/ma_links"
                 "/ma_explore/$a/action" "/ma_explore/$a/goal" "/ma_explore/$a/known_map" "/$a/plan")
    done
    setsid ros2 bag record -o "$RUN/bag" "${TOPICS[@]}" /tf /tf_static > "$RUN/bag.log" 2>&1 &
    PIDS+=($!)
fi
echo "waiting for the planners (policy warm-up, first map, Nav2) ..."
for i in $(seq 1 60); do
    c=0; for a in "${AGENTS[@]}"; do [ -e "$RUN/$a.ready" ] && c=$((c + 1)); done
    [ $c -eq $N ] && break
    sleep 2
done
[ $c -eq $N ] || echo "only $c of $N planners ready after 2 min (see $RUN/<agent>.log); going on"
echo "exploring$([ "$LIMIT" != 0 ] && echo " for $LIMIT s"): Enter or Ctrl-C ends it (the cars stop)"
T0=$(date +%s)
STOP=false
trap 'STOP=true' INT
while ! $STOP; do
    if [ -t 0 ]; then read -r -t 3 _ && break; else sleep 3; fi
    [ "$LIMIT" != 0 ] && [ $(($(date +%s) - T0)) -ge "$LIMIT" ] && break
    line=""
    for a in "${AGENTS[@]}"; do
        last=$(tail -1 "$RUN/${a}_decisions.csv" 2>/dev/null | awk -F, 'NR==1 && $2 ~ /^[0-9]+$/ {print "#" $2 " " $16}')
        line="$line  $a ${last:--}"
    done
    printf '\r  %4d s%s        ' $(($(date +%s) - T0)) "$line"
done
trap '' INT
echo
echo "ending: stopping the planners (they cancel their goals) ..."
for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done
for i in $(seq 1 20); do
    alive=0; for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [ $alive -eq 0 ] && break; sleep 0.5
done
for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null; done
{
    echo "duration: $(($(date +%s) - T0)) s"
    for a in "${AGENTS[@]}"; do
        f="$RUN/${a}_decisions.csv"
        [ -f "$f" ] || { echo "$a: no decisions"; continue; }
        echo "$a: $(($(wc -l < "$f") - 1)) decisions: $(tail -n +2 "$f" | cut -d, -f16 | sort | uniq -c | tr -s ' ' | tr '\n' ',' | sed 's/,$//'); \
failed goals: $(grep -c ' aborted;\| rejected;' "$RUN/$a.log")"
    done
} | tee "$RUN/summary.txt"
echo "results: $RUN"
}
main "$@"

#!/bin/bash
# ROBOT. Runs the robot's ORB-SLAM3 (multi-robot keyframe sharing). Options add the
# occupancy grid, keyboard driving, Nav2 navigation or autonomous exploration on top.
# Builds the orb_slam3 package (and ma_explore_ros with --explore) first.
#
# Needs on the robot: AGENT_NAME (e.g. agent_2), REALSENSE3_CONFIG (the ORB-SLAM3
# settings file of this robot's camera) and ORB_SLAM3_ROOT (the ORB_SLAM3 folder).
# Together with (on a PC, same ROS_DOMAIN_ID):
#   --teleop              ./run_teleop.sh AGENT          drive it from the keyboard
#   --nav                 ./run_rviz.sh AGENT            give goals by clicking ("2D Goal Pose")
#   --nav --ma-frames     ./run_robot_explore.sh AGENT ...   the planner runs on the PC
#   --explore             nothing (the policy runs on the robot); optional live view:
#                         ros2 run ma_explore_ros robot_view --ros-args -p agents:="AGENT"
#
#   ./run_slam.sh [--method new|hq-mpshare] [--grid] [--teleop] [--nav | --nav-wheels] [--no-bridge]
#                 [--explore [--team "A B C"] [--policy FILE] [--mission S]] [--ma-frames] [--save] [--imu] [--monitor]
#                 [--no-record]
#
#   --method      how robots share maps: new (default; keyframe sharing) or hq-mpshare
#                 (sharing of high-quality map points)
#   --grid        also publish an occupancy grid (/AGENT/map) from the depth camera
#   --teleop      keyboard driving from a PC: starts the base driver and a small gray camera view
#   --nav         --grid plus Nav2; the car's position comes from SLAM
#   --nav-wheels  as --nav, but the position comes from wheel odometry corrected by SLAM
#   --no-bridge   with --nav: after a tracking loss hold the car until SLAM's new map merges
#                 back (default: place the new map by wheel odometry and carry on, provisional
#                 until the merge)
#   --explore     --nav plus the trained exploration policy on this robot, choosing goals by
#                 itself (src/ma_explore_ros/policy/policy.pt; training puts it there).
#                 Pause it: ros2 topic pub --once /ma_explore/AGENT/enable std_msgs/msg/Bool "{data: false}"
#   --team "A B C"  with --explore: the whole team's agent names (several robots: add --ma-frames)
#   --policy FILE   with --explore: another policy file
#   --mission S     with --explore: the mission length in seconds (the policy's time input is the share of it
#                   gone, as in training; about 6 s per m^2 of free floor). Without it the input counts decisions
#   --ma-frames   several robots on one ROS domain: this robot's TF frames get its name as
#                 prefix (AGENT/map_nav, AGENT/odom, ...)
#   --save        save every keyframe's RGB + depth and the final poses (folder slam_00N in
#                 $RESULT_DIR, default ~/result)
#   --imu         use the camera's IMU (RGB-D-inertial; default: RGB-D only)
#   --monitor     log the Jetson's CPU, GPU and memory use (2 Hz; on by default while recording)
#   --no-record   do not record the run (default: every run is recorded, see "Run record" below)
#   Environment: TRACKING_CPU=5 TRACKING_RTPRIO=80 pin the tracking thread to a core.
#
# Run record (default): everything needed to analyse the run afterwards, in
# ${RESULT_DIR:-~/result}/runs/DATE_TIME_AGENT/ and, once stopped, packed into DATE_TIME_AGENT.tgz next to it:
#   console.log  everything printed, each line with its wall-clock time (ORB-SLAM3's own output too:
#                maps, merges, loop closures, keyframe sharing)
#   info.txt     command, options, machine, clock sync, the code's commits and uncommitted changes
#   ros_log/     every node's ROS log
#   eval/        the keyframe sharing's logs (alignments, verifications, keyframes served, final keyframes)
#   bag/         a rosbag of the small topics: SLAM pose and odometry, wheel odometry, IMU, the grid map,
#                velocity commands, driver heartbeat, SLAM metrics, provisional flag, Jetson load,
#                the planner's goals, actions and status (no images, no point clouds)
#   planner/     with --explore: the planner's decisions, tracking-loss events and stall snapshots
# Stop with one Ctrl-C and wait for "run saved" (stopping the nodes and packing take a few seconds).
#
# Examples:
#   ./run_slam.sh                          SLAM only
#   ./run_slam.sh --teleop --grid          drive by keyboard, watch the map
#   ./run_slam.sh --nav                    click goals in RViz
#   ./run_slam.sh --explore                one robot explores alone
#   ./run_slam.sh --explore --team "agent_1 agent_2 agent_3" --ma-frames    a team, policy on each robot
#   ./run_slam.sh --nav --ma-frames        a team, planners on the PC (run_robot_explore.sh)
if [[ "$1" == -h || "$1" == --help ]]; then awk 'NR>1{if(!/^#/)exit; sub(/^# ?/,""); print}' "$0"; exit 0; fi

SAVE_KEYFRAMES=false
MA_METHOD="new"
MONITOR=false
MONITOR_RATE=2.0
USE_IMU=false            # --imu: RGB-D-inertial (default RGB-D only: no drift while the IMU initialises)
OCCUPANCY_GRID=false
TELEOP=false
NAV=false
NAV_POSE=slam
NAV_BRIDGE=true
MA_FRAMES=false
EXPLORE=false
EXPLORE_TEAM=""
EXPLORE_POLICY=""
EXPLORE_MISSION=""
RECORD=true
MONITOR_SET=false
ORIG_ARGS="$*"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --save)
            SAVE_KEYFRAMES=true
            shift
            ;;
        --method)
            MA_METHOD="$2"
            shift 2
            ;;
        --imu)
            USE_IMU=true
            shift
            ;;
        --monitor)
            MONITOR=true
            MONITOR_SET=true
            shift
            ;;
        --grid)
            OCCUPANCY_GRID=true
            shift
            ;;
        --teleop)
            TELEOP=true
            shift
            ;;
        --nav)
            NAV=true
            shift
            ;;
        --nav-wheels)
            NAV=true
            NAV_POSE=wheels
            shift
            ;;
        --no-bridge)
            NAV_BRIDGE=false
            shift
            ;;
        --ma-frames)
            MA_FRAMES=true
            shift
            ;;
        --explore)
            EXPLORE=true
            NAV=true
            shift
            ;;
        --team)
            EXPLORE_TEAM="$2"
            shift 2
            ;;
        --policy)
            EXPLORE_POLICY="$(realpath "$2")"
            shift 2
            ;;
        --mission)
            EXPLORE_MISSION="$2"
            shift 2
            ;;
        --no-record)
            RECORD=false
            shift
            ;;
        *)
            echo "Unknown option: $1   (./run_slam.sh --help lists the options)"
            exit 1
            ;;
    esac
done

if [ ! -f "$REALSENSE3_CONFIG" ]; then
    echo "Error: ORB-SLAM3 settings file not found: $REALSENSE3_CONFIG"
    exit 1
fi
if [ -z "$AGENT_NAME" ]; then
    echo "Error: AGENT_NAME environment variable is not set."
    exit 1
fi

if [ "$EXPLORE" = "true" ]; then
    POL="${EXPLORE_POLICY:-$(cd "$(dirname "$0")" && pwd)/src/ma_explore_ros/policy/policy.pt}"
    if [ ! -f "$POL" ]; then
        echo "Error: no policy file $POL"
        echo "  copy the trained network there (src/ma_explore_ros/policy/README.md)"
        exit 1
    fi
    if ! python3 -c "import torch" 2>/dev/null; then
        echo "Error: PyTorch is not installed for $(command -v python3)."
        echo "  On a Jetson do not pip-install torch from PyPI: see 'Requirements: PyTorch on a Jetson'"
        echo "  in src/ma_explore_ros/README.md (CPU-only build, or NVIDIA's wheel for this JetPack)."
        exit 1
    fi
    N_TEAM=$(echo $EXPLORE_TEAM | wc -w)
    if [ "$N_TEAM" -gt 1 ] && [ "$MA_FRAMES" != "true" ]; then
        echo "Error: a team of several robots needs --ma-frames (each robot's TF frames named after it)"
        exit 1
    fi
fi

if [ "$MA_FRAMES" = "true" ] && [ "$NAV_POSE" = "wheels" ]; then
    # the base driver publishes plain 'odom -> base_footprint'; with prefixed frames Nav2 would not find it
    echo "Error: --ma-frames needs the SLAM pose (--nav), not --nav-wheels."
    exit 1
fi

if [ "$RECORD" = "true" ] && [ "$MONITOR_SET" = "false" ]; then
    MONITOR=true                    # the Jetson's load belongs in the run record
fi
if [ "$SAVE_KEYFRAMES" = "true" ]; then
    echo "Keyframe saving: ENABLED (result_dir=${RESULT_DIR:-\$HOME/result})"
else
    echo "Keyframe saving: disabled"
fi
echo "MA method: $MA_METHOD"
echo "IMU mode: $USE_IMU"
echo "Jetson monitor: $MONITOR (${MONITOR_RATE} Hz)"
echo "Occupancy grid: $OCCUPANCY_GRID"
echo "Teleop: $TELEOP"
echo "Navigation: $NAV (robot pose from: $NAV_POSE)"
echo "Exploration: $EXPLORE${EXPLORE_TEAM:+ (team: $EXPLORE_TEAM)}"
echo "TF frames: $([ "$MA_FRAMES" = "true" ] && echo "prefixed ($AGENT_NAME/map_nav, ...)" || echo "plain (map_nav, ...)")"
[ "$NAV" = "true" ] && echo "  Nav2 starts once the floor is calibrated (keep the floor in view, a few seconds)."

colcon build --packages-select orb_slam3 --cmake-clean-cache
if [ "$EXPLORE" = "true" ]; then colcon build --packages-select ma_explore_ros; fi
source install/setup.bash

# Plain TF frames (map_nav, odom, base_footprint) are the same names on every
# robot: with a second robot's SLAM on this ROS domain both would publish
# odom -> base_footprint, and each Nav2 would mix the two robots' poses
# (costmap "sensor origin out of map bounds", empty plans). Refuse to start.
if [ "$NAV" = "true" ] && [ "$MA_FRAMES" != "true" ]; then
    echo "Checking the ROS domain (${ROS_DOMAIN_ID:-0}) for other robots (3 s)..."
    OTHERS=$(python3 - "$AGENT_NAME" <<'EOF'
import sys, time
import rclpy
rclpy.init()
n = rclpy.create_node('run_slam_frame_check')
end = time.time() + 3.0
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
found = set()
for name, _ in n.get_topic_names_and_types():
    part = name.split('/')       # /<agent>/orb_slam3/pose: published by each robot's SLAM node
    if len(part) == 4 and part[2:] == ['orb_slam3', 'pose'] and part[1] != sys.argv[1] \
            and n.get_publishers_info_by_topic(name):
        found.add(part[1])
print(' '.join(sorted(found)))
EOF
)
    if [ -n "$OTHERS" ]; then
        echo "Error: SLAM of other robots is running on ROS_DOMAIN_ID ${ROS_DOMAIN_ID:-0}: $OTHERS"
        echo "  With plain TF frames the robots' Nav2 would mix their poses. Start every robot with"
        echo "  --ma-frames (and --team \"...\" with --explore), or give this robot its own ROS_DOMAIN_ID."
        exit 1
    fi
fi

# ros2 launch rejects an empty '<name>:=' value, so only pass result_dir when
# the user actually set RESULT_DIR (otherwise the node defaults to ~/result).
LAUNCH_ARGS=(
    agent:="$AGENT_NAME"
    vocab_file:=${ORB_SLAM3_ROOT}/Vocabulary/ORBvoc.txt
    settings_file:="$REALSENSE3_CONFIG"
    save_keyframes:="$SAVE_KEYFRAMES"
    ma_method:="$MA_METHOD"
    use_imu:="$USE_IMU"
    monitor:="$MONITOR"
    monitor_rate_hz:="$MONITOR_RATE"
    occupancy_grid:="$OCCUPANCY_GRID"
    teleop:="$TELEOP"
    nav:="$NAV"
    nav_pose:="$NAV_POSE"
    nav_bridge:="$NAV_BRIDGE"
)
# The base driver owns the motor board's serial port: if run_joystick.sh or
# run_controller.sh already started it, do not start a second one.
if [ "$TELEOP" = "true" ] || [ "$NAV" = "true" ]; then
    if pgrep -f "lib/jetracer/jetracer( |$)" >/dev/null; then
        echo "Base driver already running; not starting another"
        if [ "$NAV" = "true" ] && [ "$NAV_POSE" = "slam" ]; then
            # run_joystick.sh's EKF (and a driver with publish_odom_transform)
            # publishes odom -> base_footprint, which SLAM publishes here.
            echo "Error: --nav takes the robot pose from SLAM, but run_joystick.sh /"
            echo "run_controller.sh is running and publishes its own odom -> base_footprint."
            echo "Stop it first (or use --nav-wheels to navigate on its wheel odometry)."
            exit 1
        fi
        [ "$NAV" = "true" ] && echo "  (navigation needs odom -> base_footprint from it or its EKF)"
        LAUNCH_ARGS+=(teleop_driver:=false)
    fi
fi
if [ -n "$RESULT_DIR" ]; then
    LAUNCH_ARGS+=(result_dir:="$RESULT_DIR")
fi
if [ "$MA_FRAMES" = "true" ]; then
    LAUNCH_ARGS+=(frame_prefix:="$AGENT_NAME")
fi
if [ "$EXPLORE" = "true" ]; then
    LAUNCH_ARGS+=(explore:=true explore_policy:="$POL")
    [ -n "$EXPLORE_TEAM" ] && LAUNCH_ARGS+=(explore_team:="$EXPLORE_TEAM")
    [ -n "$EXPLORE_MISSION" ] && LAUNCH_ARGS+=(explore_mission_s:="$EXPLORE_MISSION")
fi

# Optional CPU dedication for the tracking thread (see the core-dedication
# instructions). Example: TRACKING_CPU=5 TRACKING_RTPRIO=80 ./run_slam.sh
if [ -n "$TRACKING_CPU" ]; then
    LAUNCH_ARGS+=(tracking_cpu:="$TRACKING_CPU")
fi
if [ -n "$TRACKING_RTPRIO" ]; then
    LAUNCH_ARGS+=(tracking_rtprio:="$TRACKING_RTPRIO")
fi

if [ "$RECORD" != "true" ]; then
    ros2 launch orb_slam3 orb_slam3.launch.py "${LAUNCH_ARGS[@]}"
    exit $?
fi

# ── Run record (see the header) ─────────────────────────────────────────
WS_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN_NAME="$(date +%Y%m%d_%H%M%S)_${AGENT_NAME}"
RUNS_DIR="${RESULT_DIR:-$HOME/result}/runs"
RUN_DIR="$RUNS_DIR/$RUN_NAME"
mkdir -p "$RUN_DIR/eval" "$RUN_DIR/ros_log"
START_MARK="$RUN_DIR/.start"; touch "$START_MARK"
{
    echo "run: $RUN_NAME"
    echo "command: $0 $ORIG_ARGS"
    echo "start: $(date -Iseconds)  ($(date +%s.%N))"
    echo "host: $(hostname)  agent: $AGENT_NAME  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
    echo "settings: $REALSENSE3_CONFIG"
    echo "options: method $MA_METHOD, imu $USE_IMU, grid $OCCUPANCY_GRID, teleop $TELEOP, nav $NAV ($NAV_POSE), bridge $NAV_BRIDGE," \
         "explore $EXPLORE (team: ${EXPLORE_TEAM:-alone}, mission ${EXPLORE_MISSION:-none}), ma_frames $MA_FRAMES, save $SAVE_KEYFRAMES"
    echo "--- clock"
    timedatectl 2>/dev/null | grep -i "synchronized\|NTP\|Time zone"
    chronyc tracking 2>/dev/null | grep -i "system time\|last offset\|leap"
    echo "--- code"
    for repo in "ros2_ws:$WS_DIR" "ORB_SLAM3:$ORB_SLAM3_ROOT"; do
        d="${repo#*:}"
        echo "${repo%%:*}: $(git -C "$d" log -1 --format='%h %ci %s' 2>/dev/null)"
        git -C "$d" status --short 2>/dev/null | sed 's/^/    /'
    done
    [ "$EXPLORE" = "true" ] && echo "policy: $POL  $(sha256sum "$POL" | cut -c1-16)"
    echo "--- environment"
    env | grep -E '^(MA_NEW_|TRACKING_|RMW_|ROS_)' | sort
    echo "--- machine"
    uname -a
    cat /etc/nv_tegra_release 2>/dev/null | head -1
    nproc; free -h | head -2; df -h "$HOME" | tail -1
} > "$RUN_DIR/info.txt" 2>&1
for repo in "ros2_ws:$WS_DIR" "ORB_SLAM3:$ORB_SLAM3_ROOT"; do
    git -C "${repo#*:}" diff HEAD > "$RUN_DIR/${repo%%:*}.patch" 2>/dev/null
    [ -s "$RUN_DIR/${repo%%:*}.patch" ] || rm -f "$RUN_DIR/${repo%%:*}.patch"
done
export MA_NEW_EVAL_DIR="$RUN_DIR/eval"
export ROS_LOG_DIR="$RUN_DIR/ros_log"

# The small topics (no images, no point clouds). zstd per message: the grid maps shrink ~20x.
A="/$AGENT_NAME"
BAG_TOPICS=(
    $A/orb_slam3/odom $A/orb_slam3/pose $A/orb_slam3/slam_metrics $A/orb_slam3/provisional
    $A/orb_slam3/ma_links $A/orb_slam3/merge_transform $A/orb_slam3/test_blind
    $A/odom $A/imu $A/base_driver/ok $A/cmd_vel $A/cmd_vel_gate_in $A/map
    $A/motor/lvel $A/motor/rvel $A/motor/lset $A/motor/rset $A/jetson/metrics
    $A/navigate_to_pose/_action/status
    /ma_explore$A/status /ma_explore$A/goal /ma_explore$A/action /ma_explore$A/note /ma_explore$A/enable
)
# A script's background commands start with Ctrl-C ignored, and the recorder would then not close
# its bag on our kill -INT below: give it back the default handling first.
DFL_INT='import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])'
python3 -c "$DFL_INT" ros2 bag record -o "$RUN_DIR/bag" --compression-mode message --compression-format zstd \
    "${BAG_TOPICS[@]}" > "$RUN_DIR/bag.log" 2>&1 &
BAG_PID=$!
sleep 2
if ! kill -0 $BAG_PID 2>/dev/null; then          # no zstd plugin: record uncompressed
    rm -rf "$RUN_DIR/bag"
    python3 -c "$DFL_INT" ros2 bag record -o "$RUN_DIR/bag" "${BAG_TOPICS[@]}" >> "$RUN_DIR/bag.log" 2>&1 &
    BAG_PID=$!
fi

echo "Recording this run to $RUN_DIR"
echo "  stop with ONE Ctrl-C and wait for 'run saved'"
# Ctrl-C reaches every process of this terminal: the nodes shut down, the bag closes; this script
# and the console logger carry on until the launch has ended, then pack the record.
trap 'echo; echo "[run_slam] stopping: waiting for the nodes to shut down, then saving the run ..."' INT
STAMP_PY='
import signal, sys, time
signal.signal(signal.SIGINT, signal.SIG_IGN)      # keep logging the shutdown; ends when the launch closes the pipe
f = open(sys.argv[1], "a", buffering=1, errors="replace")
out = sys.stdout.buffer
for line in iter(sys.stdin.buffer.readline, b""):
    try:
        out.write(line); out.flush()
    except Exception:
        pass
    f.write("%.3f %s" % (time.time(), line.decode("utf-8", "replace")))
'
ros2 launch orb_slam3 orb_slam3.launch.py "${LAUNCH_ARGS[@]}" 2>&1 | python3 -u -c "$STAMP_PY" "$RUN_DIR/console.log"
trap '' INT                     # from here on, finish the record whatever is pressed

kill -INT $BAG_PID 2>/dev/null
for i in $(seq 1 20); do kill -0 $BAG_PID 2>/dev/null || break; sleep 0.5; done
echo "end: $(date -Iseconds)  ($(date +%s.%N))" >> "$RUN_DIR/info.txt"
# the planner's own folder (explore.launch.py writes it to ~/result/ma_explore_robot/DATE_AGENT)
if [ "$EXPLORE" = "true" ]; then
    P=$(find "$HOME/result/ma_explore_robot" -maxdepth 1 -type d -name "*_$AGENT_NAME" -newer "$START_MARK" 2>/dev/null | sort | tail -1)
    [ -n "$P" ] && cp -r "$P" "$RUN_DIR/planner"
fi
rm -f "$START_MARK"
tar czf "$RUNS_DIR/$RUN_NAME.tgz" -C "$RUNS_DIR" "$RUN_NAME"
echo
echo "[run_slam] run saved: $RUN_DIR"
echo "           packed:    $RUNS_DIR/$RUN_NAME.tgz ($(du -h "$RUNS_DIR/$RUN_NAME.tgz" | cut -f1))"

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
#   ./run_slam.sh [--method new|hq-mpshare] [--grid] [--teleop] [--nav | --nav-wheels]
#                 [--explore [--team "A B C"] [--policy FILE]] [--ma-frames] [--save] [--imu] [--monitor]
#
#   --method      how robots share maps: new (default; keyframe sharing) or hq-mpshare
#                 (sharing of high-quality map points)
#   --grid        also publish an occupancy grid (/AGENT/map) from the depth camera
#   --teleop      keyboard driving from a PC: starts the base driver and a small gray camera view
#   --nav         --grid plus Nav2; the car's position comes from SLAM
#   --nav-wheels  as --nav, but the position comes from wheel odometry corrected by SLAM
#   --explore     --nav plus the trained exploration policy on this robot, choosing goals by
#                 itself (src/ma_explore_ros/policy/policy.pt; training puts it there).
#                 Pause it: ros2 topic pub --once /ma_explore/AGENT/enable std_msgs/msg/Bool "{data: false}"
#   --team "A B C"  with --explore: the whole team's agent names (several robots: add --ma-frames)
#   --policy FILE   with --explore: another policy file
#   --ma-frames   several robots on one ROS domain: this robot's TF frames get its name as
#                 prefix (AGENT/map_nav, AGENT/odom, ...)
#   --save        save every keyframe's RGB + depth and the final poses (folder slam_00N in
#                 $RESULT_DIR, default ~/result)
#   --imu         use the camera's IMU (RGB-D-inertial; default: RGB-D only)
#   --monitor     log the Jetson's CPU, GPU and memory use (2 Hz)
#   Environment: TRACKING_CPU=5 TRACKING_RTPRIO=80 pin the tracking thread to a core.
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
MA_FRAMES=false
EXPLORE=false
EXPLORE_TEAM=""
EXPLORE_POLICY=""
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
fi

# Optional CPU dedication for the tracking thread (see the core-dedication
# instructions). Example: TRACKING_CPU=5 TRACKING_RTPRIO=80 ./run_slam.sh
if [ -n "$TRACKING_CPU" ]; then
    LAUNCH_ARGS+=(tracking_cpu:="$TRACKING_CPU")
fi
if [ -n "$TRACKING_RTPRIO" ]; then
    LAUNCH_ARGS+=(tracking_rtprio:="$TRACKING_RTPRIO")
fi

ros2 launch orb_slam3 orb_slam3.launch.py "${LAUNCH_ARGS[@]}"

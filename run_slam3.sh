#!/bin/bash
# Build and launch the ORB-SLAM3 multi-agent SLAM node.
# The ORB-SLAM2 setup (run_slam.sh) is untouched; both can coexist.
#
# Settings note: ORB-SLAM3 needs its own settings format (File.version +
# Camera.type), so this does NOT use $REALSENSE_CONFIG (that stays for
# ORB-SLAM2). Override with REALSENSE_CONFIG_ORBSLAM3 if needed.
#
# Usage:
#   ./run_slam3.sh                 # run SLAM, do NOT save keyframes (default)
#   ./run_slam3.sh --save          # also save keyframes for offline neural SDF
#   ./run_slam3.sh --method hq-mpshare   # use the map-point-sharing method (default: new, keyframe sharing)
#   ./run_slam3.sh --save --method new   # combine flags
#   ./run_slam3.sh --grid          # also publish a Nav2 occupancy grid (/<agent>/map)
#   ./run_slam3.sh --teleop        # single-robot test: drive it from another computer's
#                                  # keyboard (there: ./run_teleop.sh <agent>); publishes a
#                                  # small gray camera view and starts the base driver
#   ./run_slam3.sh --nav           # single-robot navigation: grid + Nav2 for the car; give
#                                  # goals with RViz "2D Goal Pose" (./run_rviz.sh <agent>).
#                                  # The controller's position feedback is the SLAM pose.
#   ./run_slam3.sh --nav-wheels    # same, but position from wheel odometry corrected by SLAM
#   ./run_slam3.sh --explore       # autonomous exploration: --nav plus the trained ma_explore
#                                  # policy on this robot (src/ma_explore_ros, policy/policy.pt;
#                                  # see src/ma_explore_ros/README.md). Options: --team "r1 r2 r3"
#                                  # (the whole team, several robots: add --ma-frames),
#                                  # --policy FILE. Pause: ros2 topic pub --once
#                                  # /ma_explore/$AGENT_NAME/enable std_msgs/msg/Bool "{data: false}"
#   ./run_slam3.sh --nav --ma-frames   # several robots on one ROS domain: this robot's TF frames
#                                  # get its name as prefix (<agent>/map_nav, <agent>/odom, ...),
#                                  # as in the multi-robot simulation. For autonomous exploration
#                                  # (ma_explore on a PC: scripts/run_robot_explore.sh AGENT ...);
#                                  # RViz then: ./run_rviz.sh <agent> <agent>
#
# When saving is on, each keyframe's RGB + depth and the final optimized
# keyframe poses are written to a slam_00N folder (default under ~/result) in
# the neural-sdf-lab/rgbd_pipeline dataset format. Override the location with
# the RESULT_DIR environment variable.

SAVE_KEYFRAMES=false
MA_METHOD="new"
MONITOR=false
MONITOR_RATE=2.0
# IMU / RGBD-Inertial mode:
#   false (DEFAULT) -> plain RGBD (visual-only; no inertial-init stationary drift)
#   true            -> force RGBD-Inertial (--imu)
#   auto            -> use IMU iff the settings file has IMU.T_b_c1 calibration (--imu-auto)
USE_IMU="false"
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
        --save|--save-keyframes|save|yes|true)
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
        --imu-auto)
            USE_IMU=auto
            shift
            ;;
        --no-imu)
            USE_IMU=false
            shift
            ;;
        --monitor)
            MONITOR=true
            shift
            ;;
        --monitor-rate)
            MONITOR_RATE="$2"
            shift 2
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
            echo "Unknown argument: $1"
            echo "Usage: ./run_slam3.sh [--save] [--method new|hq-mpshare] [--imu|--imu-auto|--no-imu]"
            echo "                      [--monitor] [--monitor-rate HZ] [--grid] [--teleop] [--nav|--nav-wheels] [--ma-frames]"
            echo "                      [--explore [--team \"r1 r2 r3\"] [--policy FILE]]"
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
# instructions). Example: TRACKING_CPU=5 TRACKING_RTPRIO=80 ./run_slam3.sh
if [ -n "$TRACKING_CPU" ]; then
    LAUNCH_ARGS+=(tracking_cpu:="$TRACKING_CPU")
fi
if [ -n "$TRACKING_RTPRIO" ]; then
    LAUNCH_ARGS+=(tracking_rtprio:="$TRACKING_RTPRIO")
fi

ros2 launch orb_slam3 orb_slam3.launch.py "${LAUNCH_ARGS[@]}"

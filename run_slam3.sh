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
#
# When saving is on, each keyframe's RGB + depth and the final optimized
# keyframe poses are written to a slam_00N folder (default under ~/result) in
# the neural-sdf-lab/rgbd_pipeline dataset format. Override the location with
# the RESULT_DIR environment variable.

SAVE_KEYFRAMES=false
for arg in "$@"; do
    case "$arg" in
        --save|--save-keyframes|save|yes|true)
            SAVE_KEYFRAMES=true
            ;;
        *)
            echo "Unknown argument: $arg (use --save to enable keyframe saving)"
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

if [ "$SAVE_KEYFRAMES" = "true" ]; then
    echo "Keyframe saving: ENABLED (result_dir=${RESULT_DIR:-\$HOME/result})"
else
    echo "Keyframe saving: disabled"
fi

colcon build --packages-select orb_slam3 --cmake-clean-cache
source install/setup.bash

# ros2 launch rejects an empty '<name>:=' value, so only pass result_dir when
# the user actually set RESULT_DIR (otherwise the node defaults to ~/result).
LAUNCH_ARGS=(
    agent:="$AGENT_NAME"
    vocab_file:=${ORB_SLAM3_ROOT}/Vocabulary/ORBvoc.txt
    settings_file:="$REALSENSE3_CONFIG"
    save_keyframes:="$SAVE_KEYFRAMES"
)
if [ -n "$RESULT_DIR" ]; then
    LAUNCH_ARGS+=(result_dir:="$RESULT_DIR")
fi

ros2 launch orb_slam3 orb_slam3.launch.py "${LAUNCH_ARGS[@]}"

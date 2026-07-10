#!/bin/bash
# Build and launch the ORB-SLAM3 multi-agent SLAM node.
# The ORB-SLAM2 setup (run_slam.sh) is untouched; both can coexist.
#
# Settings note: ORB-SLAM3 needs its own settings format (File.version +
# Camera.type), so this does NOT use $REALSENSE_CONFIG (that stays for
# ORB-SLAM2). Override with REALSENSE_CONFIG_ORBSLAM3 if needed.


SETTINGS="${REALSENSE_CONFIG_ORBSLAM3:-${ORB_SLAM3_ROOT}/Examples/RGB-D/realsense.yaml}"

if [ ! -f "$SETTINGS" ]; then
    echo "Error: ORB-SLAM3 settings file not found: $SETTINGS"
    exit 1
fi
if [ -z "$AGENT_NAME" ]; then
    echo "Error: AGENT_NAME environment variable is not set."
    exit 1
fi

colcon build --packages-select orb_slam3 --cmake-clean-cache
source install/setup.bash
ros2 launch orb_slam3 orb_slam3.launch.py \
    agent:="$AGENT_NAME" \
    vocab_file:=${ORB_SLAM3_ROOT}/Vocabulary/ORBvoc.txt \
    settings_file:="$SETTINGS"

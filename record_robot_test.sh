#!/usr/bin/env bash
# PC (Vicon/base station). Record the three-robot real-world test data, including
# Vicon ground truth, into a new rosbag folder under the run's result directory.
#
# Usage:
#   ./record_robot_test.sh OUTPUT_DIR AGENT [AGENT ...]
# Example:
#   ./record_robot_test.sh ~/result/robot_test/lab3_rl_1/bag agent_1 agent_2 agent_3
#
# Start the Vicon receiver first and use the same ROS_DOMAIN_ID as the robots.
# ROS Humble's setup scripts reference optional, unset variables, so avoid
# nounset while sourcing them.
set -eo pipefail

usage() {
    sed -n '2,12p' "$0" | sed 's/^#\s\?//'
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi
if [[ $# -lt 2 ]]; then
    usage
    exit 2
fi

OUTPUT_DIR="$1"
shift
AGENTS=("$@")
WS="$(cd "$(dirname "$0")" && pwd)"

if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
    echo "Error: set ROS_DOMAIN_ID to the robots' domain first (the manual uses 10)." >&2
    exit 2
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
if [[ -f "$WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
fi

if [[ -e "$OUTPUT_DIR" ]]; then
    echo "Error: output already exists: $OUTPUT_DIR" >&2
    echo "Choose a new run folder so previous data is preserved." >&2
    exit 2
fi

TOPICS=(/tf /tf_static)
VICON_TOPICS=()
for agent in "${AGENTS[@]}"; do
    TOPICS+=(
        "/$agent/map"
        "/$agent/orb_slam3/odom"
        "/$agent/orb_slam3/slam_metrics"
        "/$agent/orb_slam3/ma_links"
        "/ma_explore/$agent/action"
        "/ma_explore/$agent/goal"
        "/$agent/plan"
        "/vicon/$agent/$agent"
    )
    VICON_TOPICS+=("/vicon/$agent/$agent")
done

VISIBLE_TOPICS="$(ros2 topic list 2>/dev/null || true)"
MISSING=()
for topic in "${VICON_TOPICS[@]}"; do
    if ! grep -Fxq "$topic" <<<"$VISIBLE_TOPICS"; then
        MISSING+=("$topic")
    fi
done

if [[ ${#MISSING[@]} -gt 0 ]]; then
    echo "Error: expected Vicon topics are not visible on ROS_DOMAIN_ID=$ROS_DOMAIN_ID:" >&2
    printf '  %s\n' "${MISSING[@]}" >&2
    echo "Start the Vicon receiver and run this recorder before starting the robots." >&2
    echo "Check the agent names, ROS_DOMAIN_ID, and Vicon subject/segment names." >&2
    exit 1
fi

mkdir -p "$(dirname "$OUTPUT_DIR")"
echo "Recording test topics for ${AGENTS[*]} on ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "Vicon ground truth: /vicon/<agent>/<agent> for each requested agent"
echo "Robot topics may appear after recording starts; start the robots now."
echo "Bag: $OUTPUT_DIR"
echo "Press Ctrl+C after the run; rosbag writes its metadata on shutdown."
exec ros2 bag record -o "$OUTPUT_DIR" "${TOPICS[@]}"

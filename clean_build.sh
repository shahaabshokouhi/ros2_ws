#!/usr/bin/env bash
# ROBOT or PC. Deletes this workspace's build/, install/ and log/ and builds every package
# again. Needed once after packages were renamed or removed (e.g. orbslam2_msgs ->
# orbslam3_msgs, 15 Oct 2026); otherwise colcon build (or the run scripts) is enough.
#
#   ./clean_build.sh
#
# Example:  git pull && ./clean_build.sh
if [[ "$1" == -h || "$1" == --help ]]; then awk 'NR>1{if(!/^#/)exit; sub(/^# ?/,""); print}' "$0"; exit 0; fi
set -e
cd "$(dirname "$0")"
rm -rf build install log
colcon build
echo "Built. In each new terminal: source install/setup.bash (the run scripts do it themselves)."

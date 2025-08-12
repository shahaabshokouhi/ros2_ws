cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select orbslam2_msgs --symlink-install
source install/setup.bash
colcon build --packages-select orb_slam2 --symlink-install
source install/setup.bash

colcon build --select-packages-from-paths src/orb_slam2 --symlink-install
source install/setup.bash
ros2 run orb_slam2 orb_slam_node

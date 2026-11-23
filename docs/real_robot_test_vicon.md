# Real robot test recording with Vicon ground truth

This supplements `~/robot_test_manual.pdf` (Three-Robot Exploration Test, 19 October 2026). It records each robot's Vicon pose in the same ROS bag as its map, SLAM odometry, exploration actions and goals. The bag stays inside the run folder under `~/result/robot_test/`, so it can be copied to another computer with the rest of the results.

## Before the run

Use the same `ROS_DOMAIN_ID` on the Vicon/base station and all robots. The manual uses `10`. On the base station, start a terminal for the Vicon receiver:

```bash
cd ~/vicon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch vicon_receiver client.launch.py
```

That launch file connects to the Vicon server at `192.168.11.2`. Keep this terminal running. Its configured namespace is `vicon`; the receiver publishes a `PoseStamped` for a subject/segment at `/vicon/<subject>/<segment>`. The current robot naming uses the same subject and segment name, so the expected ground-truth topics are:

```text
/vicon/agent_1/agent_1
/vicon/agent_2/agent_2
/vicon/agent_3/agent_3
```

The receiver stamps poses in `vicon_world` and converts Vicon translations from millimeters to meters. Its quaternion is the global segment orientation. Vicon and ROS host clocks should be synchronized for useful time alignment; the message stamp is the receiver's ROS clock at receipt.

In a second base-station terminal, start the observer as described in the manual:

```bash
export ROS_DOMAIN_ID=10
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ma_explore_ros robot_view --ros-args -p agents:="agent_1 agent_2 agent_3"
```

In a third base-station terminal, start the recorder before starting the robots, as the manual requires:

```bash
export ROS_DOMAIN_ID=10
cd ~/ros2_ws
./record_robot_test.sh ~/result/robot_test/lab3_rl_1/bag agent_1 agent_2 agent_3
```

Use a fresh run name for each run (`lab3_rl_2`, etc.). The recorder checks that the three requested Vicon topics are visible, then waits for the rest of the run. Start the robots immediately after the recorder reports that it is recording. The explicitly selected map, SLAM and exploration topics are included as their publishers appear. If the script reports missing Vicon topics, check that the receiver is connected, the subject and segment names match the agent names, and the Vicon terminal uses the same ROS domain.

Stop recording with Ctrl+C after stopping the robots. This lets rosbag finish its metadata. The result folder will contain `bag/metadata.yaml` and the bag storage file; Vicon is stored as `/vicon/<agent>/<agent>` in `geometry_msgs/msg/PoseStamped` messages, synchronized by their header timestamps. Copy the complete `bag/` directory along with the robot logs described in Section 8 of the manual. On another ROS 2 computer, inspect it with:

```bash
ros2 bag info ~/result/robot_test/lab3_rl_1/bag
ros2 bag play ~/result/robot_test/lab3_rl_1/bag
```

The ground-truth samples are available while playing on each `/vicon/<agent>/<agent>` topic. This gives the raw Vicon trajectory for later time alignment and analysis; it does not transform Vicon poses into each robot's ORB-SLAM map frame.

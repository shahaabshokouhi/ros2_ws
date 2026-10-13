# ma_explore_ros: autonomous exploration on the robot

The trained ma_explore policy runs on the robot itself: each robot's planner
builds the policy's observation from its own SLAM grid, pose and SLAM state
(and, once merged, its teammates'), picks the next goal (a frontier, keep,
WAIT, or REVISIT a merged teammate's pose) and sends it to its Nav2.

## Run

    # one robot (plain frames)
    ./run_slam3.sh --explore
    # several robots on one ROS domain: on each robot, the whole team named
    ./run_slam3.sh --explore --ma-frames --team "robot1 robot2 robot3"

`--explore` is `--nav` (SLAM, grid, Nav2) plus this planner; it starts
exploring once Nav2 is up and the floor is calibrated. Pause and resume:

    ros2 topic pub --once /ma_explore/$AGENT_NAME/enable std_msgs/msg/Bool "{data: false}"

Watch from a PC (same ROS_DOMAIN_ID): `ros2 run ma_explore_ros robot_view --ros-args -p agents:="robot1"`.
Results on the robot: `~/result/ma_explore_robot/<date>_<time>_<agent>/`
(`<agent>_decisions.csv`: every decision and its kind; `<agent>_slam_events.csv`:
every tracking loss, where, the heading, the goal's bearing, how long).

## After a tracking loss

The SLAM node's safety gate stops the car and cancels the goal. The planner
waits 2 s, then backs the car up slowly along the last 0.8 m it drove (wheel
odometry), so the camera sees again what it saw before the loss and ORB-SLAM3
can relocalize; up to 2 attempts, then it stops and logs that it needs help
(drive it back with the keyboard). Parameters: recovery, recovery_wait,
recovery_back_m, recovery_speed, recovery_attempts. Every loss and recovery is a
row of `<agent>_slam_events.csv`.

## Policy

`policy/policy.pt` (see policy/README.md). `ros2 run ma_explore_ros policy_info`
prints which training run it comes from and times a decision.

## Requirements

PyTorch for the robot's ROS Python (3.10): `pip3 install --user torch`
(the CPU wheel for aarch64 is enough: one decision every few seconds).

## Where the code comes from

The modules in `ma_explore_ros/` (env, geometry, model, live, baselines,
policy_io, ros_planner, robot_view) are copies of the training repository
(~/ma_explore on the training PC), made by its `scripts/sync_robot_package.sh`;
`ma_explore_ros/SYNCED_FROM` names the commit. Edit them there, not here: the
live observation must stay identical to the training one.

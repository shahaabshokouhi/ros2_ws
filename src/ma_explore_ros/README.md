# ma_explore_ros: autonomous exploration on the robot

The trained ma_explore policy runs on the robot itself: each robot's planner
builds the policy's observation from its own SLAM grid, pose and SLAM state
(and, once merged, its teammates'), picks the next goal (a frontier, keep,
WAIT, or REVISIT a merged teammate's pose) and sends it to its Nav2.

## Run

    # one robot (plain frames)
    ./run_slam.sh --explore
    # several robots on one ROS domain: on each robot, the whole team named
    ./run_slam.sh --explore --ma-frames --team "robot1 robot2 robot3"

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

## When the car is stuck

If Nav2 aborts 3 goals within 40 s while the car moves less than 0.15 m, its
costmaps hold obstacles all around it (in Isaac: teammates that drove close by
stay marked beside and behind the car after they have left, since only the
forward camera can erase a mark, and the car's Nav2 recoveries never clear the
costmaps). The planner then (1) clears the local and global costmaps and
decides again; (2) if it is stuck again within 2 min, retraces the last 0.3 m it
drove (in reverse, or forwards if that stretch was driven in reverse: ground it
has just driven over), clears again and decides; (3) otherwise logs that it
needs help. Parameters: unstick, unstick_aborts, unstick_window, unstick_move,
unstick_back, unstick_reset; test with unstick_test_at:=40.0 (acts as if stuck
40 s and 55 s after the start). Events stuck_clear / stuck_backup / stuck_help
go to `<agent>_slam_events.csv`.

## Policy

`policy/policy.pt` (see policy/README.md). `ros2 run ma_explore_ros policy_info`
prints which training run it comes from and times a decision.

## Requirements: PyTorch on a Jetson

Do **not** `pip3 install torch` from PyPI on a Jetson: recent aarch64 PyPI
wheels are CUDA builds for server ARM (SBSA), not for the Jetson's integrated
GPU and JetPack, and pip may also upgrade numpy to 2.x, which breaks ROS 2
Humble's Python packages (built against numpy 1.x). PyTorch must be importable
by the Python that ROS 2 runs (`/usr/bin/python3`, 3.10 on JetPack 6 / Ubuntu 22.04).

First check what the robot has:

    cat /etc/nv_tegra_release                      # L4T release (R36.x = JetPack 6.x)
    apt-cache show nvidia-jetpack 2>/dev/null | grep -m1 Version
    python3 -c "import numpy; print(numpy.__version__)"

Then one of:

* **A. CPU-only PyTorch (enough for this planner, independent of JetPack).**
  The planner runs on the CPU (`device:=cpu`, the default), about one
  decision every few seconds. PyTorch's CPU-only build has no CUDA, so it does
  not need to match JetPack:

      pip3 install --user --index-url https://download.pytorch.org/whl/cpu "torch==2.4.*"
      python3 -c "import numpy; print(numpy.__version__)"   # must still be 1.x

  If numpy was upgraded, put it back: `pip3 install --user "numpy<2"`.

* **B. NVIDIA's PyTorch for your JetPack (if you also want the GPU).**
  Follow NVIDIA's "Installing PyTorch for Jetson Platform" guide and take the
  wheel built for exactly your JetPack and Python (cp310 for JetPack 6), from
  NVIDIA's index (developer.download.nvidia.com/compute/redist/jp/<v60|v61|...>/pytorch/)
  or NVIDIA's "PyTorch for Jetson" forum page; install its listed prerequisites
  first (e.g. `libopenblas-dev`; newer builds also need cuSPARSELt). Install
  the wheel with `pip3 install --user <file>.whl` and check numpy as in A.
  Then the planner can use `device:=cuda`, though the CPU is fast enough.

Check: `ros2 run ma_explore_ros policy_info` prints the PyTorch and numpy
versions, whether CUDA is available, which network is installed and how long a
decision takes on this computer.

## Where the code comes from

The modules in `ma_explore_ros/` (env, geometry, model, live, baselines,
policy_io, ros_planner, robot_view) are copies of the training repository
(~/ma_explore on the training PC), made by its `python -m ma_explore.robot_package --modules-only`;
`ma_explore_ros/SYNCED_FROM` names the commit. Edit them there, not here: the
live observation must stay identical to the training one.

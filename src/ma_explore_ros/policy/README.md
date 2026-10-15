# The trained network

`policy.pt` is the policy the robots run (`./run_slam.sh --explore`). A finished training run
on the PC copies it here by itself (ma_explore `train.publish_to_robot`, on for
`configs/slam_aware.yaml`), together with the planner code it was trained with
(`../ma_explore_ros/`), and writes `POLICY_FROM` (which checkpoint, how many steps). Then
commit and push ros2_ws on the PC, and `git pull` on each robot: no rebuild is needed for
the policy itself.

By hand, on the PC: `python -m ma_explore.robot_package RUN_DIR` (in ~/ma_explore).
Check what is installed: `ros2 run ma_explore_ros policy_info`.

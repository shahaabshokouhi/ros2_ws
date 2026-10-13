# The trained network

Copy the network to use into this folder as `policy.pt`:

    scp <PC>:~/result/ma_explore/runs/<run>/checkpoints/robot_policy.pt \
        ~/ros2_ws/src/ma_explore_ros/policy/policy.pt    # the robots: ~/ros2_ws

`robot_policy.pt` is written at the end of every training run (the best
checkpoint's actor and settings, about 2.5 MB); the training log and the run's
report (`<run>/report/report.html`) say which file it is. A full `best.pt` works
too. No rebuild is needed after copying. Check what is installed:

    ros2 run ma_explore_ros policy_info

`*.pt` files are not committed (this folder's .gitignore).

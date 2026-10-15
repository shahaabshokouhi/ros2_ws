# Which script is for what

Every script prints what it does, where it runs, what must run with it, and all its options
with examples: `./SCRIPT.sh --help`. All of them run from this folder (`ros2_ws`). The
training itself is the only script outside it: `~/ma_explore/train.sh`.

## On the robot (JetRacer)

| Script | What it does | Runs with (on a PC, same `ROS_DOMAIN_ID`) |
|---|---|---|
| `run_slam.sh` | **The main one.** ORB-SLAM3. Options: `--grid` occupancy grid, `--teleop` keyboard driving, `--nav` Nav2 (goals from RViz), `--explore` the trained policy chooses goals on the robot, `--ma-frames` several robots on one domain, `--method`, `--save`, `--imu`, `--monitor`. | `--teleop`: `run_teleop.sh`; `--nav`: `run_rviz.sh`; `--nav --ma-frames`: `run_robot_explore.sh`; `--explore`: nothing |
| `run_joystick.sh` | Gamepad driving (no Nav2). Not with `run_slam.sh --nav`/`--explore`. | — |
| `run_controller.sh` | Vicon PID waypoint follower. Not with `run_slam.sh --nav`/`--explore`. | the Vicon bridge |
| `reset_camera.sh` | USB reset of a stuck RealSense ("Frames didn't arrive"). | — |
| `clean_build.sh` | Delete build/install/log and build everything again (once after `git pull` of the 15 Oct 2026 rename). | — |

## On the PC with real robots (station)

| Script | What it does | Robot side |
|---|---|---|
| `run_rviz.sh AGENT [PREFIX]` | RViz for one robot; goals with "2D Goal Pose". | `run_slam.sh --nav` (or `--grid`) |
| `run_teleop.sh AGENT` | Drive one robot from the keyboard (opens RViz too). | `run_slam.sh --teleop` |
| `run_robot_explore.sh AGENT ...` | Exploration with the planners on the PC, with a live view. Needs `~/ma_explore`. | `run_slam.sh --nav` (several robots: `--nav --ma-frames`) |

## Simulation (Isaac Sim, on the PC; ROS domain 31)

| Script | What it does |
|---|---|
| `run_baselines.sh` | **The stored baselines:** the published methods (`--mmpf --mtsp --burgard --alc`, default all), several sessions each, in the lab; the reference every policy is compared with. |
| `run_isaac_explore.sh` | **Exploration test:** three cars with SLAM, Nav2 and an ma_explore planner (the trained policy, rules, or the published methods), in the lab (default) or the room; full report. Needs `~/ma_explore`. |
| `run_ma_sim_test.sh` | **SLAM test:** several cars drive fixed routes in the room; scored against ground truth. |
| `run_ma_sim_nav.sh` | Several cars in the room with Nav2; you give goals in RViz; report at the end. |
| `run_isaac_sim.sh` | Isaac Sim with cars, nothing else (the others start it). By hand: one car with `run_sim_slam.sh`, or to look at a world. |
| `run_sim_slam.sh` | The robot stack for the one car of `run_isaac_sim.sh` (the simulation's `run_slam.sh`). |

## The trained policy on the robots

A finished training run (`~/ma_explore/train.sh`) copies its best policy to
`src/ma_explore_ros/policy/policy.pt`, together with the planner code it was trained with.
Commit and push this workspace; `git pull` on each robot; `./run_slam.sh --explore`.

Offline evaluation of the sharing method on recorded data lives in the ORB_SLAM3 repository
(`~/ORB_SLAM3/eval/kf_sharing/run_replay.sh`).

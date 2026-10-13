# Which script is for what

Every script starts with a comment that explains it in full; this is the map.

## On a robot (JetRacer)

| Script | What it does |
|---|---|
| `run_slam3.sh` | **The main one.** ORB-SLAM3 multi-agent SLAM. Options add, in this order of use: `--grid` (occupancy grid), `--teleop` (drive from a PC keyboard), `--nav` (grid + Nav2, goals from RViz), `--explore` (Nav2 + the trained ma_explore policy choosing goals on its own; policy in `src/ma_explore_ros/policy/policy.pt`), `--ma-frames` (several robots on one ROS domain), `--method new\|hq-mpshare`, `--save`, `--imu`. |
| `run_joystick.sh` | Manual driving with a joystick (base driver, EKF on wheels + IMU). Not with `--nav`/`--explore`. |
| `run_controller.sh` | Vicon PID waypoint follower (lab with Vicon). Not with `--nav`/`--explore`. |
| `reset_camera.sh` | Software replug of a stuck RealSense ("Frames didn't arrive"). |
| `run_slam.sh` | ORB-SLAM2 node (the earlier SLAM). |
| `build_run.sh`, `clean_build.sh` | Old ORB-SLAM2 helpers with old paths; kept for reference. |

## On a PC (same `ROS_DOMAIN_ID` as the robot)

| Script | What it does |
|---|---|
| `run_rviz.sh AGENT [PREFIX]` | RViz for one robot: map, pose, Nav2 plan, costmaps; give goals with "2D Goal Pose". |
| `run_teleop.sh AGENT` | Drive one robot from this keyboard (robot: `run_slam3.sh --teleop`). |
| `ros2 run ma_explore_ros robot_view --ros-args -p agents:="AGENT"` | Live view of what an exploring robot knows and decides. |

## Simulation (Isaac Sim, on the PC)

| Script | What it does |
|---|---|
| `run_isaac_sim.sh` + `run_sim_slam.sh` | One simulated JetRacer: start Isaac first, then the robot stack on it. |
| `run_ma_sim_nav.sh` | Several simulated cars, each with SLAM and Nav2, goals by hand in RViz; ground-truth report at the end. (A spec with one robot does not work: Isaac names it `sim`; use two.) |
| `run_ma_sim_test.sh` | Several simulated cars driving fixed routes; SLAM accuracy and merging report. |

Exploration sessions with the trained policy in Isaac, and training itself, are in the
ma_explore repository on the training PC (`scripts/run_isaac_explore.sh`,
`scripts/run_robot_explore.sh`, `scripts/sync_robot_package.sh`; its README).

# ros2_ws — multi-robot SLAM, mapping and navigation for JetRacer robots

ROS 2 Humble workspace for Shahab Shokouhi's multi-agent SLAM research. Each
robot (a JetRacer with a RealSense D435i) runs ORB-SLAM3 through the node in
this workspace. Robots can share maps with each other, build an occupancy grid
from depth, and navigate with Nav2. An Isaac Sim stand-in runs the same stack
on a simulated car.

The SLAM library itself, including all multi-robot map sharing, lives in the
companion repository **ORB_SLAM3**: `~/ORB_SLAM3`
([github.com/shahaabshokouhi/ORB_SLAM3](https://github.com/shahaabshokouhi/ORB_SLAM3/tree/master)).
Its README explains the library, the multi-robot methods and their `MA_NEW_*`
settings, the build scripts and the offline evaluation harness. Both repos
have their work on their main branch (ORB_SLAM3: **`master`**, ros2_ws: **`main`**). Always pull and rebuild **both**
on every machine: a robot running an older message layout or library silently
drops the other robots' messages.

---

## Contents

1. [What do I run? (recipes)](#1-what-do-i-run-recipes)
2. [One-time setup](#2-one-time-setup)
3. [Scripts reference](#3-scripts-reference)
4. [Launch files reference](#4-launch-files-reference)
5. [SLAM node parameters](#5-slam-node-parameters)
6. [Topics and frames](#6-topics-and-frames)
7. [How the occupancy grid works](#7-how-the-occupancy-grid-works)
8. [How navigation works](#8-how-navigation-works)
9. [How the Isaac Sim stand-in works](#9-how-the-isaac-sim-stand-in-works)
10. [Packages and files](#10-packages-and-files)
11. [Troubleshooting](#11-troubleshooting)

---

## 1. What do I run? (recipes)

All commands run from `~/workspaces/ros2_ws`. "Robot" means the Jetson on the
car; "PC" means the desktop. Scripts that launch ROS rebuild the package they
need first.

| I want to… | On the robot | On the PC |
|---|---|---|
| Run SLAM on one robot | `./run_slam3.sh` | `./run_rviz.sh <agent>` to watch |
| Run the multi-robot keyframe-sharing method (the research method) | `./run_slam3.sh --method new` on **every** robot | |
| Run the original map-point-sharing method | `./run_slam3.sh` (or `--method hq-mpshare`) on every robot | |
| Get an occupancy grid for planning | `./run_slam3.sh --grid` | `./run_rviz.sh <agent>` |
| Drive one robot from the PC keyboard | `./run_slam3.sh --teleop` | `./run_teleop.sh <agent>` (opens RViz too) |
| Send one robot to a point (Nav2) | `./run_slam3.sh --nav` (add `--teleop` to also see its camera) | `./run_rviz.sh <agent>`, then the **2D Goal Pose** button |
| Same, but navigate on wheel odometry | `./run_slam3.sh --nav-wheels` | same |
| Save keyframes for offline reconstruction | `./run_slam3.sh --save` | |
| Drive with a gamepad (no SLAM) | `./run_joystick.sh` | |
| Follow Vicon waypoints | `./run_controller.sh` | |
| Fix a wedged RealSense ("Frames didn't arrive") | `./reset_camera.sh` | |
| Test the whole stack in Isaac Sim | — | terminal 1 `./run_isaac_sim.sh`; terminal 2 `./run_sim_slam.sh --nav --teleop`; then `ROS_DOMAIN_ID=31 ./run_rviz.sh sim` and/or `ROS_DOMAIN_ID=31 ./run_teleop.sh sim` |
| Evaluate the sharing method offline on recorded data | — | `~/ORB_SLAM3/eval/kf_sharing/run_replay.sh` ([its README](https://github.com/shahaabshokouhi/ORB_SLAM3/blob/master/eval/kf_sharing/README.md)) |

Flags can be combined, for example `./run_slam3.sh --method new --grid --teleop`.
`--teleop` and `--nav` are **single-robot** features: they start the motor
driver and publish robot-specific frames. Use them on one robot at a time.

---

## 2. One-time setup

### Environment (`~/.bashrc` on every machine)

| Variable | Used by | Example | Meaning |
|---|---|---|---|
| `AGENT_NAME` | `run_slam3.sh`, `run_slam.sh`, `run_joystick.sh`, `run_controller.sh`, `run_rviz.sh` (default) | `agent_2` / `shahab` | This robot's name. It prefixes every per-robot topic (`/<agent>/…`) and is the SLAM node's name. Must be unique per robot. |
| `REALSENSE3_CONFIG` | `run_slam3.sh` | `~/realsense3.yaml` | ORB-SLAM3 settings file with **this camera's** calibration (ORB-SLAM3 format: `File.version`, `Camera.type`). |
| `ORB_SLAM3_ROOT` | `run_slam3.sh` | `~/ORB_SLAM3` | Where the ORB_SLAM3 repo is (for the vocabulary). |
| `ROS_DOMAIN_ID` | all ROS processes | `10` | All machines that should see each other use the same value. The simulation uses its own (31, see `SIM_DOMAIN`). |
| `REALSENSE_CONFIG`, `ORB_SLAM2_ROOT` | `run_slam.sh` (legacy ORB-SLAM2 only) | `~/realsense.yaml` | ORB-SLAM2-format settings and repo. |
| `RESULT_DIR` | `run_slam3.sh --save` | `~/result` | Where `--save` writes its `slam_00N` dataset folders (default `~/result`). |
| `TRACKING_CPU`, `TRACKING_RTPRIO` | `run_slam3.sh` | `5`, `80` | Optional: pin the tracking thread to one CPU core and give it real-time priority. |
| `SIM_DOMAIN` | `run_isaac_sim.sh`, `run_sim_slam.sh` | `31` (default) | ROS domain of the simulated robot, kept apart from the real robots. |

### Build

```bash
# 1. the SLAM library (see the ORB_SLAM3 README; first time: ./build.sh)
cd ~/ORB_SLAM3 && ./rebuild.sh
# 2. this workspace
cd ~/workspaces/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select orbslam2_msgs orb_slam3 jetracer jetracer_sim
```

`run_slam3.sh` rebuilds `orb_slam3` itself each time. After a `git pull` that
changed messages (`src/orbslam2_msgs`), rebuild `orbslam2_msgs` too, on every
machine.

### Extra packages

| For | Install |
|---|---|
| Navigation (`--nav`) | `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup` |
| Simulation (`jetracer_sim`) | `sudo apt install ros-humble-ackermann-msgs`; Isaac Sim with Isaac Lab, activated by `~/activate_isaaclab_ros2.sh` |
| Gamepad (`run_joystick.sh`) | `ros-humble-joy`, `ros-humble-robot-localization` |

If `apt install ros-humble-navigation2` fails with *libgoogle-glog-dev: Depends:
libunwind-dev … not installable*, LLVM's C++ library is in the way. Remove it
first (nothing in these repos uses it):
`sudo apt remove libc++-dev libc++-14-dev libunwind-14-dev`.

---

## 3. Scripts reference

### `run_slam3.sh` — the robot's main entry point (ORB-SLAM3)

Builds the `orb_slam3` package and launches
`orb_slam3/launch/orb_slam3.launch.py` with the RealSense driver and the SLAM
node, plus whatever the flags add.

| Flag | Effect |
|---|---|
| *(none)* | RealSense + ORB-SLAM3 (RGB-D) + multi-robot map sharing with method `hq-mpshare`. |
| `--method new` | Multi-robot method: `hq-mpshare` (share high-quality map points; the default) or `new` (keyframe sharing, the current research method). Every robot in a run must use the same method. See the ORB_SLAM3 README. |
| `--imu` / `--imu-auto` / `--no-imu` | RGB-D-inertial (`--imu`), inertial only if the settings file has `IMU.T_b_c1` (`--imu-auto`), or plain RGB-D (`--no-imu`, the default; inertial initialisation drifts while standing still). |
| `--save` | Also save every keyframe's RGB, depth and final pose as a `slam_00N` dataset under `$RESULT_DIR` (default `~/result`), for offline reconstruction. |
| `--monitor`, `--monitor-rate HZ` | Also run the Jetson hardware monitor (`/<agent>/jetson/metrics`: CPU, GPU, RAM, power, temperatures), default 2 Hz. |
| `--grid` | Occupancy grid from depth for planning (`/<agent>/map`). See [§7](#7-how-the-occupancy-grid-works). |
| `--teleop` | Single robot: publish a small gray camera view (`/<agent>/orb_slam3/gray`, 320×240, 5 Hz, about 0.35 MB/s) and start the motor driver, so `run_teleop.sh` on the PC can drive the car. |
| `--nav` | Single robot: grid + Nav2 (car planner and controller) + motor driver; the robot's position comes from SLAM. Goals from RViz. See [§8](#8-how-navigation-works). |
| `--nav-wheels` | Like `--nav`, but the position comes from wheel odometry, corrected by SLAM. |

Notes:

* `--teleop` and `--nav` start the motor driver (`jetracer` node, serial port
  `/dev/ttyACM0`) only if `run_joystick.sh` / `run_controller.sh` is not
  already running one. With `--nav` (SLAM pose) the script **refuses** to
  start while those scripts run, because their pose filter publishes the same
  frame SLAM does. Stop them first, or use `--nav-wheels`.
* `--nav` waits a few seconds for the floor calibration (keep the floor in
  view). Until then Nav2 prints `Timed out waiting for transform … odom`,
  which is normal.
* Any `MA_NEW_*` environment variable is passed to the SLAM node, for example
  `MA_NEW_VERBOSE=1 ./run_slam3.sh --method new` (list in the ORB_SLAM3 README).

### `run_rviz.sh [AGENT]` — watch one robot (PC)

Opens RViz with `robot_view.rviz`, filled in for the given agent (default
`$AGENT_NAME`). Fixed frame `map_nav`. It shows the occupancy grid, the
robot's pose and path, the camera view (with `--teleop`), the Nav2 plan, the
local costmap, the car's outline and the live obstacle scan. Some displays
start switched off (ORB-SLAM3 map points, global costmap). Its **2D Goal
Pose** button sends Nav2 goals: click the target, drag for the final heading.

Needs the robot's `ROS_DOMAIN_ID`. Most displays need `--grid` or `--nav` on
the robot; before the floor calibration, RViz reports that `map_nav` does not
exist.

### `run_teleop.sh AGENT [param:=value ...]` — keyboard driving (PC)

Opens RViz (`run_rviz.sh`) in the background and turns the terminal into a
keyboard controller (`ros2 run jetracer teleop`), publishing
`/<AGENT>/cmd_vel` at 20 Hz. The robot needs `run_slam3.sh --teleop` (or
`--nav`). Single robot only.

| Key | Action |
|---|---|
| `w` / `s` or ↑ / ↓ | forward / backward |
| `q` / `e` | forward while turning left / right |
| `z` / `c` | backward while turning left / right |
| `a` / `d` or ← / → | turn the front wheels only (a car cannot turn on the spot) |
| `space` or `x` | stop |
| `+` / `-` | faster / slower |
| `Ctrl-C` | stop the car and quit |

**Hold** the key: the command drops to zero 0.7 s after the last key press,
and the motor driver stops the car on its own after 1 s without commands
(e.g. if Wi-Fi drops). Parameters (as `param:=value`): `speed` (0.15 m/s),
`turn` (0.8 rad/s), `max_linear` (0.3), `max_angular` (1.5), `hold_timeout`
(0.7 s), `latch` (false; true = a key keeps its command until `space`).
Teleop goes straight to the motors: it is **not** stopped by Nav2's safety
gate, so it can always rescue a robot. Keep your hands off the keys while
Nav2 drives.

### `run_isaac_sim.sh [options]` — the simulated robot (PC, terminal 1)

Runs `src/jetracer_sim/isaac/run_isaac.py` with Isaac Sim's own Python
(`~/activate_isaaclab_ros2.sh`) on domain `$SIM_DOMAIN` (31). The GUI opens
by default; the simulation starts playing once the terminal prints
`[jetracer_sim] running …`. See [§9](#9-how-the-isaac-sim-stand-in-works).

The window opens on a camera in the room's upper south-west corner, behind
the car's start. Five fixed views are in the viewport's camera menu (camera
icon, top left of the viewport), under `/World/SimViews`: `Top` (a flat plan
of the whole room, x to the right and y up, like RViz) and `Corner_NE`,
`Corner_NW`, `Corner_SW`, `Corner_SE` (upper corners looking at the room
centre; N = +y, E = +x). `Perspective` is the free camera again.

| Option | Default | Effect |
|---|---|---|
| `--headless` | off | no window |
| `--agent NAME` | `sim` | robot name (topic prefix) |
| `--usd FILE` | `~/Jetracer/jetracer3.usd` | scene |
| `--no-realtime` | off | run as fast as possible (default: paced to real time; images use the computer's clock) |
| `--plain-room` | off | leave the room as the `.usd` has it: bare walls, sun light, no ceiling (default: furnished for visual SLAM, see [§9](#9-how-the-isaac-sim-stand-in-works)) |
| `--light I` | 12000 | brightness of each of the 9 ceiling panels; with `--plain-room`: an even dome light (default 1000, 0 = none), without which the bare room is dark beyond ~1 m |
| `--max-steer RAD` | 0.59 | steering limit (the real car: 0.6) |
| `--width`, `--height` | 640, 480 | camera resolution (the ORB-SLAM3 settings assume 640×480) |

### `run_sim_slam.sh [options]` — the robot's stack on the simulated car (PC, terminal 2)

Builds `orb_slam3`, `jetracer_sim`, `jetracer` and launches
`jetracer_sim/launch/sim.launch.py`: the same `orb_slam3.launch.py` the robot
uses, without the RealSense and motor drivers, plus a stand-in for the motor
driver. Start `run_isaac_sim.sh` first.

| Option | Effect |
|---|---|
| *(none)* | SLAM + occupancy grid |
| `--nav` | + Nav2 (position from SLAM), as `run_slam3.sh --nav` |
| `--nav-wheels` | + Nav2 on "wheel odometry", which in the simulation is the true pose |
| `--teleop` | gray camera view for `run_teleop.sh` |
| `--no-grid` | no occupancy grid |
| `--agent NAME` | robot name (must match `run_isaac_sim.sh`) |

Then on the PC: `ROS_DOMAIN_ID=31 ./run_rviz.sh sim` and/or
`ROS_DOMAIN_ID=31 ./run_teleop.sh sim`.

### Other scripts

| Script | What it does |
|---|---|
| `run_joystick.sh` | Gamepad driving without SLAM: `jetracer.launch.py` (joy node, `teleop_joy`, motor driver, wheel+IMU EKF, OLED display). Hold the dead-man button (button index 6) to drive. |
| `run_controller.sh` | Vicon waypoint follower: `pid_controller.launch.py` (PID on the Vicon pose `/vicon/<agent>/<agent>`, waypoints from `jetracer/config/waypoints.yaml`, or goals published to it with `goal_subscriber:=true`, which this script sets). |
| `reset_camera.sh` | Software unplug/replug of the RealSense (USB reset). Wait about 4 s afterwards. |
| `run_slam.sh` | **Legacy:** the ORB-SLAM2 node (`orb_slam2` package) with `$REALSENSE_CONFIG`, `$ORB_SLAM2_ROOT`. Wire-compatible with ORB-SLAM3 robots running `hq-mpshare`. |
| `build_run.sh` | **Legacy:** ORB-SLAM2 with hard-coded paths of the old `jetson1` robot. |
| `clean_build.sh` | **Legacy, not in git:** wipes `build/ install/ log/` and rebuilds the ORB-SLAM2 packages (assumes `~/ros2_ws`). |
| `~/ORB_SLAM3/eval/kf_sharing/run_replay.sh` | Offline evaluation: replays a recorded RGB-D sequence into several SLAM nodes on an isolated domain (77) and scores them against ground truth. Documented in the ORB_SLAM3 repo. |

---

## 4. Launch files reference

### `orb_slam3/launch/orb_slam3.launch.py`

What `run_slam3.sh` (and, without camera and driver, `run_sim_slam.sh`) runs.

| Argument | Default | Meaning |
|---|---|---|
| `agent` | `agent_0` | robot name; node name and topic prefix |
| `vocab_file`, `settings_file` | — | ORB vocabulary, ORB-SLAM3 camera settings |
| `ma_method` | `hq-mpshare` | multi-robot method: `hq-mpshare` or `new` |
| `use_imu` | `false` | `false` / `true` / `auto` |
| `save_keyframes`, `result_dir` | `false`, `` | `--save` |
| `tracking_cpu`, `tracking_rtprio` | `-1`, `0` | pin / prioritise the tracking thread |
| `monitor`, `monitor_rate_hz` | `true`, `2.0` | Jetson monitor (`run_slam3.sh` passes `false` unless `--monitor`) |
| `occupancy_grid` | `false` | `--grid` |
| `teleop` | `false` | `--teleop`: gray view + motor driver |
| `nav` | `false` | `--nav`: grid + navigation frames + motor driver + Nav2 |
| `nav_pose` | `slam` | `slam` or `wheels` (`--nav-wheels`) |
| `teleop_driver` | `true` | start the motor driver with teleop/nav (`run_slam3.sh` sets `false` when another script already runs one) |
| `port_name` | `/dev/ttyACM0` | motor board serial port |
| `camera` | `true` | start the RealSense driver (`false` in simulation) |
| `extra_params` | `` | a YAML file of SLAM node parameters applied last (the simulation uses it for its camera position) |
| `sigterm_timeout`, `sigkill_timeout` | `300`, `60` | how long Ctrl-C waits for the final bundle adjustment and exports |

With `nav:=true` it also starts the Nav2 nodes (planner, controller, velocity
smoother, behaviours, BT navigator, lifecycle manager) itself, in the robot's
namespace but on the global `/tf`. Nav2's own `navigation_launch.py` would
move TF into the namespace, where nothing publishes it.

### `jetracer_sim/launch/sim.launch.py`

Arguments `agent` (`sim`), `ma_method` (`new`), `grid` (`true`), `nav`
(`false`), `nav_pose` (`slam`), `teleop` (`false`). Includes
`orb_slam3.launch.py` with `camera:=false teleop_driver:=false monitor:=false`,
the simulated camera's settings (`jetracer_sim/config/isaac_d455.yaml`) and
the sim car's camera position (`config/sim_params.yaml`), plus the
`cmd_vel_to_ackermann` node.

### `jetracer` launch files

| File | Starts | Arguments |
|---|---|---|
| `jetracer.launch.py` | joy node, `teleop_joy.py`, motor driver, EKF (`robot_localization`), OLED display, IMU static TF | `use_sim_time`, `agent_name` |
| `pid_controller.launch.py` | Vicon PID waypoint follower + motor driver | `use_sim_time`, `agent_name`, `waypoints_file`, `goal_subscriber` |
| `joy.launch.py` | joy node + `teleop_joy.py` only | `use_sim_time` |

---

## 5. SLAM node parameters

Set through the launch file, or `extra_params`. The defaults are right for the
JetRacer.

### General

| Parameter | Default | Meaning |
|---|---|---|
| `ma_method` | `hq-mpshare` | multi-robot method |
| `use_imu` | `false` | `false` / `true` / `auto` |
| `save_keyframes`, `result_dir`, `output_dir` | `false`, `` , `.` | keyframe dataset; where shutdown exports go |
| `tracking_cpu`, `tracking_rtprio` | `-1`, `0` | tracking thread pinning / priority |
| `occupancy_grid` | `false` | build the grid |
| `nav_frames` | `false` | navigation frame layout ([§6](#tf-frames)); implies the grid |
| `publish_gray` | `false` | gray camera view; `gray_rate_hz` (5), `gray_width` (320) |

### Occupancy grid and navigation (`grid.*`)

| Parameter | Default | Meaning |
|---|---|---|
| `grid.max_range` | 1.0 m | obstacles and free space only this close to the camera |
| `grid.obstacle_min_height`, `grid.obstacle_max_height` | 0.03, 0.20 m | height band above the floor that counts as an obstacle (robot 15 cm + margin) |
| `grid.below_floor` | 0.03 m | points further below the floor are reflections, ignored |
| `grid.min_points` | 3 | depth points needed (within 5 cm) to call a direction blocked |
| `grid.edge_jump`, `grid.pixel_stride` | 0.05, 3 | flying-pixel filter; pixel sampling |
| `grid.resolution` | 0.05 m | cell size |
| `grid.period` | 1.0 s | how often the map is redrawn |
| `grid.scan_interval`, `grid.scan_move`, `grid.scan_turn_deg` | 0.5 s, 0.05 m, 5° | a scan is stored at most this often and only after moving or turning this much |
| `grid.scan_rate` | 10 Hz | live obstacle scan rate |
| `grid.occupied_logodds`, `grid.free_logodds` | 1.0, -0.3 | thresholds (occupied needs two hits) |
| `grid.map_margin` | 2.0 m | unknown margin around what was seen |
| `grid.map_half_size` | 0 (10 with `nav`) | the map covers at least ±this around the start |
| `grid.nominal_camera_height` | 0.05 m | sanity check for the floor fit |
| `grid.mount_from_params`, `grid.camera_height`, `grid.camera_pitch_deg`, `grid.camera_roll_deg` | false, 0.05, 0, 0 | skip the floor fit and use these instead |
| `grid.camera_forward`, `grid.camera_left` | 0.21, 0.0 m | camera position relative to the rear axle (the robot's reference point) |
| `grid.pose_source` | `slam` | with `nav_frames`: `slam` or `wheels` |
| `grid.global_frame`, `grid.base_frame`, `grid.scan_frame`, `grid.odom_frame` | `map_nav`, `base_footprint`, `camera_floor`, `odom` | frame names |

### Multi-robot settings (`MA_NEW_*`)

The keyframe-sharing method is tuned with environment variables, not
parameters. They are listed in the
[ORB_SLAM3 README](https://github.com/shahaabshokouhi/ORB_SLAM3/blob/master/README.md#multi-robot-settings-ma_new_).

---

## 6. Topics and frames

### Per-robot topics (`/<agent>/…`)

| Topic | Type | From | When |
|---|---|---|---|
| `camera/realsense2_camera/color/image_raw`, `…/depth/image_rect_raw` | Image | RealSense (or Isaac) | always (depth aligned to colour; 16UC1 mm, or 32FC1 m from Isaac) |
| `orb_slam3/pose`, `orb_slam3/path` | PoseStamped, Path | SLAM node | while tracking (frame `map`) |
| `orb_slam3/merged_map` | PointCloud2 | SLAM node | own + shared map points |
| `orb_slam3/slam_metrics` | Float64MultiArray | SLAM node | per frame: tracking ms, state, map size… |
| `map` | OccupancyGrid | SLAM node | `--grid`/`--nav`, 1 Hz, frame `map_nav` |
| `orb_slam3/odom` | Odometry | SLAM node | `--grid`/`--nav`: SLAM pose of the rear axle in `map_nav` |
| `orb_slam3/scan` | LaserScan | SLAM node | `--grid`/`--nav`: live 1 m obstacle scan (frame `camera_floor`) |
| `orb_slam3/gray` | Image (mono8) | SLAM node | `--teleop` |
| `cmd_vel` | Twist | teleop, or Nav2 through the safety gate | → motor driver |
| `cmd_vel_gate_in` | Twist | Nav2 | `--nav`: Nav2's commands before the safety gate |
| `odom`, `imu` | Odometry, Imu | motor driver | wheel odometry and IMU |
| `goal_pose`, `plan`, `local_costmap/…`, `global_costmap/…` | Nav2 | Nav2 | `--nav` |
| `jetson/metrics` | | Jetson monitor | `--monitor` |
| `drive`, `ground_truth/odom`, `ground_truth/tf` | | Isaac Sim | simulation only |

### Robot-to-robot topics (global)

`/orb_slam2/mappoints`, `/orb_slam2/single_mappoint` (hq-mpshare);
`/orb_slam2/kf_bow`, `/orb_slam2/agent_request`, `/orb_slam2/camera_calib`,
`/orb_slam2/kf_data`, `/orb_slam2/owner_update`,
`/orb_slam2/back_observations` (new). The `orb_slam2` names are kept on
purpose: ORB-SLAM2 and ORB-SLAM3 robots can share in a mixed fleet. Message
definitions are in `src/orbslam2_msgs/msg`.

### TF frames

A TF frame has exactly one parent, so there are two layouts:

```
default:                map ─▶ camera_color_optical_frame
  --grid adds:            camera_color_optical_frame ─▶ base_footprint, camera_floor;  map ─▶ map_nav
--nav / --nav-wheels:   map ─▶ map_nav ─▶ odom ─▶ base_footprint ─▶ camera_color_optical_frame, camera_floor
```

* `map` is ORB-SLAM3's world (the first camera pose, ROS axes). It is not
  level if the camera is tilted.
* `map_nav` is level: the floor under the rear axle at the start. It's the
  global frame for the grid and Nav2.
* `base_footprint` is the floor under the **rear axle**, the point a car
  turns about.
* `camera_floor` is the floor under the camera, the origin of the scans.
* With `--nav`, `odom → base_footprint` **is** the SLAM pose and
  `map_nav → odom` is the identity. With `--nav-wheels`,
  `odom → base_footprint` is the wheel odometry, and SLAM publishes the
  correction `map_nav → odom`.

---

## 7. How the occupancy grid works

Code: `src/orb_slam3/src/occupancy_mapper.hpp`.

1. **Floor calibration.** The camera's height, pitch and roll are fitted to
   the floor in the first depth images (RANSAC). This needs the floor in view
   1–30 cm below the camera, as on the car. The node logs
   `[grid] camera mount fitted to the floor: height … pitch … roll …`; while
   it can't find the floor, it says why every 5 s.
2. **Virtual scans.** Every few centimetres of motion, depth between 3 and
   20 cm above the floor and within 1 m becomes a fan of 1° directions: the
   nearest obstacle, or how far the floor is seen free. A direction needs 3
   agreeing depth points, and pixels on depth edges are ignored. Each scan is
   stored **relative to its SLAM keyframe**.
3. **Map.** Once a second every scan is redrawn from its keyframe's *current*
   pose (log-odds; free along each ray, occupied at a hit). Corrections from
   SLAM (bundle adjustment, loop closure, map merges) therefore move the
   obstacles along, instead of leaving doubled walls.

The map lives in SLAM's main map, or before that has formed, in SLAM's first
map. So it appears immediately.

---

## 8. How navigation works

Config: `src/orb_slam3/config/nav2_jetracer.yaml`,
`navigate_car.xml`, `navigate_through_poses_car.xml`.

* **The car:** front-wheel steering, wheelbase 0.145 m, steering limit
  0.6 rad (minimum turning radius 0.21 m), 0.24 × 0.17 m, rear axle 3 cm from
  the back, camera 0.21 m ahead of the rear axle, may reverse, max 0.3 m/s.
* **Planner:** Smac Hybrid-A* with Reeds–Shepp motions. It plans over
  position and heading using car motions only, including reversing, and is
  collision-checked with the car's outline. It plans with a 0.35 m radius, so
  the controller has margin.
* **Controller:** Regulated Pure Pursuit, 0.15 m/s cruise, slower near
  obstacles, reverses when the path does, never turns on the spot.
* **Unseen space counts as free.** The map covers ±10 m around the start from
  the beginning, so a goal anywhere in it is planned at once, straight through
  what hasn't been seen. The live scan and the grid mark obstacles. The path
  is checked 3 times a second, and a blocked path is replanned.
* **Recoveries:** back up 25 cm, then wait. They never clear the costmaps:
  with a 1 m sensor that made the car forget an obstacle it could no longer
  see up close. There's no spin.
* **Safety gate:** Nav2's commands go through the SLAM node
  (`cmd_vel_gate_in → cmd_vel`). It sends zero while the SLAM pose is more
  than 0.3 s old (tracking lost), because TF would otherwise keep handing out
  the last pose and Nav2 would drive blind. Keyboard teleop is not gated. If
  the car stops this way, back it away with teleop until SLAM relocalizes.
* **Giving goals:** RViz **2D Goal Pose** (frame `map_nav`), or the
  `/<agent>/navigate_to_pose` action.

---

## 9. How the Isaac Sim stand-in works

```
 run_isaac_sim.sh (Isaac's Python)                    run_sim_slam.sh (ROS 2 Humble)
 ┌────────────────────────────────────┐   images      ┌───────────────────────────────────────────┐
 │ run_isaac.py + jetracer3.usd       │──────────────▶│ orb_slam3_node   (same code as the robot)  │
 │  ROS_Camera graph: RGB, depth, info│   true pose   │ Nav2             (same config)             │
 │  ROS_Odom graph ───────────────────│──────────────▶│ cmd_vel_to_ackermann (motor driver's       │
 │  ROS_Ackermann_Drive ◀── /sim/drive│◀──────────────│   stand-in): cmd_vel → drive; true → odom  │
 │  physics 60 Hz, real time          │               └───────────────────────────────────────────┘
 └────────────────────────────────────┘        both on ROS_DOMAIN_ID 31, plain DDS
```

* There is no bridge process: Isaac's `isaacsim.ros2.bridge` extension
  publishes and subscribes on DDS from OmniGraph nodes that run on every
  simulation tick.
* `run_isaac.py` opens the scene and adjusts it:
  * moves the camera's near clipping plane past the D455 housing model;
  * removes the camera's separate physics body;
  * furnishes the room (`room_dressing.py`, skipped with `--plain-room`):
    the scene's room is bare plaster walls under a strong sun light, where
    the camera, 5 cm above the floor, sees mostly flat grey and cannot track.
    It adds textured overlays on the four walls (posters, whiteboard, big
    stencilled zone letters, skirting, outlets, a door; every wall
    different, so places can be recognised), a textured concrete floor over
    the repeating checker, shelves with books and boxes, cabinets, crates
    and box stacks along the walls (static colliders, the middle of the room
    stays free), and a ceiling with 9 panel lights in place of the sun light.
    Textures are drawn with PIL from fixed seeds (same room every run) and
    cached in `~/.cache/jetracer_sim/textures_v1` (delete it after editing a
    texture). Nothing is written to the `.usd`;
  * with `--plain-room`, adds the dome light instead;
  * adds the five viewing cameras (`views.py`) and shows `Corner_SW`;
  * points the scene's drive and odometry graphs at the robot's topic names
    (odometry becomes **ground truth**, off `/tf`);
  * adds the camera graph (RGB + depth from the same camera, 30 Hz, system
    clock);
  * steps the physics in real time.
* `cmd_vel_to_ackermann` turns `cmd_vel` into a steering angle
  (atan(wheelbase · ω / v), clamped) and speed, stops after 1 s without
  commands, and republishes the true odometry as `/sim/odom`.
* Simulated car: wheelbase 0.150 m, camera 0.18 m ahead of the rear axle and
  6.15 cm high (`config/sim_params.yaml`). Camera 640×480, fx = fy = 317.04
  (`config/isaac_d455.yaml`). Room 6 × 6 m.
* Start-up: Nav2 is ready a few seconds after `run_sim_slam.sh`, once the
  floor is calibrated. The simulated car starts at the room centre facing +x.
  A toy truck and a mug sit about 0.5 m ahead on the left.
* Never save the scene from the Isaac window after a run (answer "Don't
  save" on close): everything above is applied in memory on every start, and
  a saved copy carries the run's additions into the file. To change the
  scene itself, open the `.usd` in plain Isaac Sim.

---

## 10. Packages and files

| Path | What |
|---|---|
| `src/orb_slam3/` | The ROS 2 node around the ORB-SLAM3 library (`src/orb_slam3_node.cpp`). Camera input, pose and map output, robot-to-robot messaging, keyframe dataset saving, evaluation logs. `src/occupancy_mapper.hpp`: grid, scans, navigation frames, safety gate. `launch/orb_slam3.launch.py`, `config/` (Nav2). |
| `src/orbslam2_msgs/` | Messages shared by all robots (map points, keyframe adverts and data, ownership updates…). |
| `src/jetracer/` | The car: motor/IMU driver `src/jetracer.cpp` (`/<agent>/cmd_vel` in, `odom`/`imu` out, stops after 1 s without commands); `scripts/teleop_keyboard.py` (`teleop`), `teleop_joy.py`, `pid_controller.py` (Vicon waypoints), `display_node.py` (OLED), `odom_ekf.py`; `jetracer/jetson_monitor.py`; `config/waypoints.yaml`. |
| `src/jetracer_sim/` | Isaac Sim stand-in: `isaac/run_isaac.py`, `isaac/room_dressing.py`, `isaac/views.py`, `jetracer_sim/cmd_vel_to_ackermann.py`, `launch/sim.launch.py`, `config/`. |
| `src/orb_slam2/` | Legacy ORB-SLAM2 node. |
| `robot_view.rviz` | RViz layout used by `run_rviz.sh` (`__AGENT__` is replaced). |

Run artefacts (`*mappoint_descriptors.csv`, `build/`, `install/`, `log/`) are
not tracked.

---

## 11. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Robots don't see each other's maps | Different `ROS_DOMAIN_ID`, different `--method`, or one machine not pulled/rebuilt (message layouts must match: rebuild `orbslam2_msgs` + `orb_slam3`, and the ORB_SLAM3 library, everywhere). |
| `Frames didn't arrive within 5 seconds` | `./reset_camera.sh`, wait 4 s, restart. |
| `frame [map_nav] does not exist` in RViz | Floor not calibrated yet: the camera must see the floor 1–30 cm below it. The node logs why every 5 s (`[grid] still calibrating the floor: …`). On the desk the camera is too high; put it on the floor. |
| Nav2 prints `Timed out waiting for transform … odom` | Normal for the first seconds (floor calibration). If it persists, check the line above. |
| Car stops during navigation, log says `[nav] no SLAM pose …` | Tracking lost: the safety gate holds the car. Back it away with teleop until SLAM relocalizes. |
| `run_slam3.sh --nav` refuses to start | `run_joystick.sh` / `run_controller.sh` is running and publishes its own robot pose. Stop it, or use `--nav-wheels`. |
| Simulation: black camera image or tracking lost after ~1 m | Use the defaults of `run_isaac_sim.sh` (clipping fix, furnished room and lights are applied automatically). |
| Simulation: Isaac exits at start, or the camera topic has 2 publishers / stray `/drive`, `/odom`, `/tf` | The `.usd` was saved after a run (possibly with the scene added twice). Restore the original file; see the note in [§9](#9-how-the-isaac-sim-stand-in-works). |
| Simulation topics invisible | Use `ROS_DOMAIN_ID=31` (or `$SIM_DOMAIN`) in every terminal that talks to the simulation. |

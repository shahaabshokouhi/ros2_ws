import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Give the SLAM node enough time to finish bundle adjustment and CSV export
        # before being force-killed. Override on CLI: ros2 launch ... sigterm_timeout:=600
        DeclareLaunchArgument('sigterm_timeout', default_value='300'),
        DeclareLaunchArgument('sigkill_timeout', default_value='60'),
        DeclareLaunchArgument(
            'agent',
            default_value='agent_0',
            description='Agent name used as topic prefix and SLAM node name (e.g., agent_0)'
        ),
        DeclareLaunchArgument(
            'vocab_file',
            default_value='/path/to/ORBvoc.txt',
            description='Path to the ORB vocabulary file'
        ),
        DeclareLaunchArgument(
            'settings_file',
            default_value='/path/to/Settings.yaml',
            description='Path to the camera settings file (ORB-SLAM3 format)'
        ),
        DeclareLaunchArgument(
            'save_keyframes',
            default_value='false',
            description='Save keyframe RGB/depth + optimized poses to a '
                        'slam_00N dataset (neural-sdf-lab/rgbd_pipeline format)'
        ),
        DeclareLaunchArgument(
            'result_dir',
            default_value='',
            description='Root folder for slam_00N datasets. Empty => $HOME/result'
        ),
        DeclareLaunchArgument(
            'tracking_cpu',
            default_value='-1',
            description='Pin the tracking thread to this CPU core; -1 = no pin'
        ),
        DeclareLaunchArgument(
            'tracking_rtprio',
            default_value='0',
            description='SCHED_FIFO priority for the tracking thread; 0 = normal'
        ),
        DeclareLaunchArgument(
            'ma_method',
            default_value='hq-mpshare',
            description='Multi-agent method: hq-mpshare (map point sharing) or new (BoW sharing)'
        ),
        DeclareLaunchArgument(
            'use_imu',
            default_value='false',
            description='RGBD-Inertial: false (DEFAULT, plain RGBD), true (force), or '
                        'auto (use IMU iff the settings file has IMU.T_b_c1). Default is '
                        'visual-only because inertial init drifts when stationary. When '
                        'not false, the RealSense driver publishes a united /imu topic.'
        ),
        DeclareLaunchArgument(
            'monitor',
            default_value='true',
            description='Launch the Jetson hardware monitor node '
                        '(publishes /<agent>/jetson/metrics at monitor_rate_hz)'
        ),
        DeclareLaunchArgument(
            'monitor_rate_hz',
            default_value='2.0',
            description='Rate at which jetson_monitor publishes hardware metrics (Hz)'
        ),
        DeclareLaunchArgument(
            'occupancy_grid',
            default_value='false',
            description='Publish a Nav2 occupancy grid (/<agent>/map, frame map_nav) '
                        'built from depth, plus base_link and /<agent>/orb_slam3/odom'
        ),
        DeclareLaunchArgument(
            'teleop',
            default_value='false',
            description='Single-robot keyboard driving from another computer: publish a '
                        'small grayscale view (/<agent>/orb_slam3/gray) and run the base '
                        'driver that executes /<agent>/cmd_vel'
        ),
        DeclareLaunchArgument(
            'teleop_driver',
            default_value='true',
            description='With teleop: also start the jetracer base driver (set false if '
                        'run_joystick.sh / run_controller.sh already runs it)'
        ),
        DeclareLaunchArgument(
            'nav',
            default_value='false',
            description='Single-robot navigation: occupancy grid, REP-105 frames '
                        '(map_nav -> odom -> base_footprint), the base driver with its '
                        'wheel odometry, and Nav2 (Hybrid-A* + pure pursuit for a car). '
                        'Goals: RViz 2D Goal Pose on /<agent>/goal_pose'
        ),
        DeclareLaunchArgument(
            'nav_pose',
            default_value='slam',
            description='With nav: where the robot pose for Nav2 comes from. slam: '
                        'odom -> base_footprint is the SLAM pose (the base driver only '
                        'drives and reports speed); wheels: wheel odometry, corrected by SLAM'
        ),
        DeclareLaunchArgument(
            'camera',
            default_value='true',
            description='Start the RealSense driver (false when the images come from '
                        'elsewhere, e.g. Isaac Sim through jetracer_sim)'
        ),
        DeclareLaunchArgument(
            'extra_params',
            default_value='',
            description='Optional YAML file of orb_slam3_node parameters applied on top '
                        '(e.g. grid.camera_forward for a different car)'
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyACM0',
            description='Serial port of the jetracer motor board (base driver)'
        ),
        OpaqueFunction(function=launch_nodes),
    ])

def launch_nodes(context):
    agent        = LaunchConfiguration('agent').perform(context)
    vocab_file   = LaunchConfiguration('vocab_file').perform(context)
    settings     = LaunchConfiguration('settings_file').perform(context)
    save_kf_str  = LaunchConfiguration('save_keyframes').perform(context)
    result_dir   = LaunchConfiguration('result_dir').perform(context)
    save_keyframes  = save_kf_str.strip().lower() in ('true', '1', 'yes', 'on')
    tracking_cpu    = int(LaunchConfiguration('tracking_cpu').perform(context))
    tracking_rtprio = int(LaunchConfiguration('tracking_rtprio').perform(context))
    ma_method       = LaunchConfiguration('ma_method').perform(context)
    use_imu         = LaunchConfiguration('use_imu').perform(context)
    enable_imu      = use_imu.strip().lower() != 'false'
    monitor_str     = LaunchConfiguration('monitor').perform(context)
    launch_monitor  = monitor_str.strip().lower() in ('true', '1', 'yes', 'on')
    monitor_rate    = float(LaunchConfiguration('monitor_rate_hz').perform(context))
    occupancy_grid  = LaunchConfiguration('occupancy_grid').perform(context).strip().lower() \
        in ('true', '1', 'yes', 'on')
    teleop          = LaunchConfiguration('teleop').perform(context).strip().lower() \
        in ('true', '1', 'yes', 'on')
    teleop_driver   = LaunchConfiguration('teleop_driver').perform(context).strip().lower() \
        in ('true', '1', 'yes', 'on')
    port_name       = LaunchConfiguration('port_name').perform(context)
    camera          = LaunchConfiguration('camera').perform(context).strip().lower() \
        in ('true', '1', 'yes', 'on')
    extra_params    = LaunchConfiguration('extra_params').perform(context).strip()
    nav             = LaunchConfiguration('nav').perform(context).strip().lower() \
        in ('true', '1', 'yes', 'on')
    nav_pose        = LaunchConfiguration('nav_pose').perform(context).strip().lower()
    if nav_pose not in ('slam', 'wheels'):
        raise RuntimeError(f"nav_pose must be slam or wheels, not '{nav_pose}'")

    def tgt(suffix: str) -> str:
        # build "/<agent>/<suffix>"
        return f"/{agent}/{suffix.lstrip('/')}"

    rs_remaps = [
        ('/camera/realsense2_camera/color/image_raw',
         tgt('camera/realsense2_camera/color/image_raw')),
        ('/camera/realsense2_camera/color/camera_info',
         tgt('camera/realsense2_camera/color/camera_info')),
        # Use the depth-to-color aligned topic, not the raw unaligned depth
        ('/camera/realsense2_camera/aligned_depth_to_color/image_raw',
         tgt('camera/realsense2_camera/depth/image_rect_raw')),
    ]
    if enable_imu:
        # United accel+gyro stream the SLAM node subscribes to for IMU_RGBD.
        rs_remaps.append(('/camera/realsense2_camera/imu',
                          tgt('camera/realsense2_camera/imu')))

    # IMU streams for RGBD-Inertial. unite_imu_method=2 (linear interpolation)
    # makes the driver publish a single combined /imu topic (accel+gyro per
    # message), which the SLAM node subscribes to. Empty dict => no IMU streams.
    imu_params = {
        'enable_gyro': True,
        'enable_accel': True,
        'unite_imu_method': 2,
    } if enable_imu else {}

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        parameters=[{
            # Power-cycle the camera in software at startup: recovers the
            # wedged stream state ("Frames didn't arrive within 5 seconds")
            # that otherwise requires physically replugging the camera.
            'initial_reset': True,
            'enable_depth': True,
            # Infra streams are unused by RGB-D SLAM; disabling them reduces
            # USB bandwidth and stream-start flakiness on old firmware.
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_color': True,
            'align_depth.enable': True,  # creates aligned_depth_to_color topic
            # Old param names (honored by older realsense2_camera drivers, as
            # on the robots):
            'color_width':  640,
            'color_height': 480,
            'depth_width':  640,
            'depth_height': 480,
            'color_fps': 30.0,
            'depth_fps': 30.0,
            # New param names (realsense2_camera >= 4.5x IGNORES the ones
            # above and streams its default 1280x720 profile otherwise —
            # which silently breaks SLAM calibrated for 640x480):
            'rgb_camera.color_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',

            # --- Motion blur during turns ---
            # Auto-exposure is left ON so the image never goes dark. Its only
            # weakness is that in a dim room it lengthens exposure, and a long
            # exposure smears the image while the robot rotates -> tracking
            # loses its features. realsense2_camera 4.58 has no "auto-exposure
            # priority" cap, so the fixes are: (a) add light to the room (auto
            # exposure then picks a short, sharp shutter by itself), or (b)
            # switch the COLOR sensor to manual exposure below.
            #
            # To use manual exposure: run `realsense-viewer`, open the Color
            # stream > Controls, turn OFF "Enable Auto Exposure", and lower
            # "Exposure" until the image is sharp while you wave the camera but
            # still bright enough to see texture (raise "Gain" a little if it
            # gets too dark). Read those two numbers off, then uncomment and
            # set them here. Exposure is in microseconds; gain is 16-248.
            # 'rgb_camera.enable_auto_exposure': False,
            # 'rgb_camera.exposure': 200,
            # 'rgb_camera.gain': 64,

            'publish_tf': False,
            **imu_params,
        }],
        remappings=rs_remaps,
    )

    # The SLAM node constructs subscriptions using get_name() as the agent
    # prefix, so the node name must equal the agent name.
    slam_node = Node(
        package='orb_slam3',
        executable='orb_slam3_node',
        name=agent,  # node name == agent
        output='screen',
        parameters=[
            {'vocab_file': vocab_file},
            {'settings_file': settings},
            {'save_keyframes': save_keyframes},
            {'result_dir': result_dir},
            {'tracking_cpu': tracking_cpu},
            {'tracking_rtprio': tracking_rtprio},
            {'ma_method': ma_method},
            {'use_imu': use_imu},
            {'occupancy_grid': occupancy_grid or nav},
            {'nav_frames': nav},
            {'grid.map_half_size': 10.0 if nav else 0.0},   # nav: a 20 x 20 m map from the start
            {'grid.pose_source': nav_pose},
            {'publish_gray': teleop},
        ] + ([extra_params] if extra_params else []),
    )

    nodes = ([realsense_node] if camera else []) + [slam_node]

    if launch_monitor:
        monitor_node = Node(
            package='jetracer',
            executable='jetson_monitor',
            name=f'{agent}_jetson_monitor',
            output='screen',
            parameters=[{
                'agent_name': agent,
                'rate_hz': monitor_rate,
            }],
        )
        nodes.append(monitor_node)

    if (teleop or nav) and teleop_driver:
        # Base driver: executes /<agent>/cmd_vel (keyboard teleop or Nav2) and
        # stops the motors by itself after 1 s without a command. Only with
        # nav_pose:=wheels does it also publish odom -> base_footprint (with
        # slam, SLAM publishes that frame; two publishers would fight).
        nodes.append(Node(
            package='jetracer',
            executable='jetracer',
            name='jetracer',
            output='screen',
            parameters=[
                {'port_name': port_name},
                {'publish_odom_transform': nav and nav_pose == 'wheels'},
                {'agent_name': agent},
            ],
        ))

    if nav:
        nodes += nav2_nodes(agent)

    return nodes


def nav2_nodes(agent):
    """Nav2 for one car, in the robot's namespace but on the global /tf.

    nav2_bringup's navigation_launch.py remaps /tf to a namespaced tf, where
    nothing publishes in this system, so the nodes are started here instead.
    """
    share = get_package_share_directory('orb_slam3')
    with open(os.path.join(share, 'config', 'nav2_jetracer.yaml')) as f:
        text = f.read()
    text = text.replace('__AGENT__', agent).replace(
        '__BT_XML_POSES__', os.path.join(share, 'config', 'navigate_through_poses_car.xml')).replace(
        '__BT_XML__', os.path.join(share, 'config', 'navigate_car.xml'))
    params = os.path.join(tempfile.gettempdir(), f'nav2_{agent}.yaml')
    with open(params, 'w') as f:
        f.write(text)

    def nav_node(pkg, exe, remaps=()):
        return Node(package=pkg, executable=exe, name=exe, namespace=agent,
                    output='screen', parameters=[params], remappings=list(remaps))

    return [
        nav_node('nav2_planner', 'planner_server'),
        nav_node('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        # Commands reach /<agent>/cmd_vel through the SLAM node's safety gate
        # (cmd_vel_gate_in), which stops the car while the SLAM pose is stale.
        nav_node('nav2_velocity_smoother', 'velocity_smoother',
                 [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_gate_in')]),
        nav_node('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_gate_in')]),
        nav_node('nav2_bt_navigator', 'bt_navigator'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', namespace=agent, output='screen',
             parameters=[params]),
    ]

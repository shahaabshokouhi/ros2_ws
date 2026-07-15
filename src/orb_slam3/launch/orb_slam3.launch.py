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
        OpaqueFunction(function=launch_nodes),
    ])

def launch_nodes(context):
    agent        = LaunchConfiguration('agent').perform(context)
    vocab_file   = LaunchConfiguration('vocab_file').perform(context)
    settings     = LaunchConfiguration('settings_file').perform(context)
    save_kf_str  = LaunchConfiguration('save_keyframes').perform(context)
    result_dir   = LaunchConfiguration('result_dir').perform(context)
    save_keyframes = save_kf_str.strip().lower() in ('true', '1', 'yes', 'on')

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
            'publish_tf': False,
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
        ],
    )

    return [realsense_node, slam_node]

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
            description='Path to the camera settings file'
        ),
        OpaqueFunction(function=launch_nodes),
    ])

def launch_nodes(context):
    agent        = LaunchConfiguration('agent').perform(context)
    vocab_file   = LaunchConfiguration('vocab_file').perform(context)
    settings     = LaunchConfiguration('settings_file').perform(context)

    def tgt(suffix: str) -> str:
        # build "/<agent>/<suffix>"
        return f"/{agent}/{suffix.lstrip('/')}"

    rs_remaps = [
        ('/camera/realsense2_camera/color/image_raw',
         tgt('camera/realsense2_camera/color/image_raw')),
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
            'enable_depth': True,
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_color': True,
            'align_depth.enable': True,  # correct param name; creates aligned_depth_to_color topic
            'color_width':  640,
            'color_height': 480,
            'depth_width':  640,
            'depth_height': 480,
            'color_fps': 30.0,
            'depth_fps': 30.0,
        }],
        remappings=rs_remaps,
    )

    # Your SLAM node constructs subscriptions using get_name() as the agent prefix.
    # So set the node name to the agent to match the remapped camera topics.
    slam_node = Node(
        package='orb_slam2',
        executable='orb_slam_node',
        name=agent,  # node name == agent
        output='screen',
        parameters=[
            {'vocab_file': vocab_file},
            {'settings_file': settings},
        ],
        # no remaps needed if your C++ builds "/<agent>/camera/..." using get_name()
    )

    return [realsense_node, slam_node]

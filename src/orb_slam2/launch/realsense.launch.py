from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def make_nodes(context):
    agent = LaunchConfiguration('agent').perform(context)

    def tgt(suffix: str) -> str:
        # builds "/<agent>/<suffix>"
        return f"/{agent}/{suffix.lstrip('/')}"

    # Remap main color/depth topics published by realsense2_camera
    remaps = [
        ('/camera/realsense2_camera/color/image_raw',       tgt('/camera/realsense2_camera/color/image_raw')),
        ('/camera/realsense2_camera/depth/image_rect_raw',  tgt('/camera/realsense2_camera/depth/image_rect_raw')),
        # Add more if you use them:
        # ('/camera/aligned_depth_to_color/image_raw', tgt('camera/aligned_depth_to_color/image_raw')),
        # ('/camera/infra1/image_rect_raw', tgt('camera/infra1/image_rect_raw')),
        # ('/camera/infra2/image_rect_raw', tgt('camera/infra2/image_rect_raw')),
    ]

    return [Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        parameters=[{
            'enable_depth': True,
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_color': True,
            'align_depth': True,
        }],
        remappings=remaps,
    )]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'agent',
            default_value='robot_0',
            description='Agent prefix for remapped topics (e.g., robot_0, robot_1)'
        ),
        OpaqueFunction(function=make_nodes),
    ])

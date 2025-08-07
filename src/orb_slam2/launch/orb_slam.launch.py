from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_name',
            default_value='orb_slam_node',
            description='Name of the ORB-SLAM2 node'
        ),
        DeclareLaunchArgument(
            'vocab_file',
            default_value='/path/to/ORBvoc.txt',
            description='Path to the ORB vocabulary file'
        ),
        DeclareLaunchArgument(
            'settings_file',
            default_value='/path/to/Settings.yaml',
            description='Path to the settings file'
        ),
        Node(
            package='orb_slam2',
            executable='orb_slam_node',
            name=LaunchConfiguration('node_name'),
            parameters=[{
                'vocab_file': LaunchConfiguration('vocab_file'),
                'settings_file': LaunchConfiguration('settings_file')
            }],
            output='screen',
        )
    ])
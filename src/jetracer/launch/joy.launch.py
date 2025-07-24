from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),

        # Joystick Node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Teleop node
        Node(
            package='jetracer',
            executable='teleop_joy.py',
            name='teleop_joy',
            output='screen',
            parameters=[
                {'x_speed': 0.3},
                {'y_speed': 0.0},
                {'w_speed': 1.0},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        )
    ])
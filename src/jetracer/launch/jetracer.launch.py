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
        ),
        
        Node(
            package = 'jetracer',
            executable = 'jetracer',
            name = 'jetracer',
            output = 'screen',
            parameters = [
                {'use_sim_time': False},
                {'port_name': '/dev/ttyACM0'},
                {'publish_odom_transform': False}
            ],
            remappings = [
            	('/odom', '/odom_raw')
            ]
        ),

        # EKF Localization Node (ros2 built-in package)
        Node(
            package = 'robot_localization',
            executable = 'ekf_node',
            name = 'robot_pose_ekf',
            output = 'screen',
            parameters = [
                {'use_sim_time': False},
                # Frame settings
                {'world_frame': 'odom'},
                {'base_link_frame': 'base_footprint'},
                # Update frequency and timeouts
                {'frequency': 30.0},
                {'sensor_timeout': 0.5},
                # Odometry source
                {'odom0': '/odom_raw'},
                {'odom0_config': [True, True, False,
                                  False, False, False]}, 
                {'use_odometry': True},
                # IMU source
                {'imu0': '/imu'},
                {'imu0_config': [False, False, False,
                                  True, True, True]},
                {'use_imu': True},
                # Disable visual odometry
                {'vo0': ''},
                {'use_vo': False},
                # Debug 
                {'debug': False},
                {'self_diagnostics': False},
            ],
            remappings = [
                ('/odom_raw', '/odom_raw'), 
                ('imu', '/imu'),    
                ('/odometry/filtered', '/odom_combined')
            ],
        ),
        
        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            name = 'base_footprint_to_imu',
            output = 'screen',
            arguments = [
                '0', '0', '0.02',  # x, y, z
                '0', '0', '0',  # roll, pitch, yaw
                'base_footprint',  # parent frame
                'base_umu_link'  # child frame
            ]
        )
    ])

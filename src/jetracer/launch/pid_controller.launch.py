from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # --- Launch arguments ---
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),

        DeclareLaunchArgument(
            'agent_name',
            default_value='agen_0',
            description='Vicon subject/segment name used in /vicon/<agent>/<agent> topic'
        ),

        # --- Vicon PID waypoint follower (replaces joystick teleop) ---
        Node(
            package='jetracer',          # <<< CHANGE THIS
            executable='pid_controller.py',  # <<< CHANGE IF NEEDED
            name='pid_controller',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'agent_name': LaunchConfiguration('agent_name')},
                # You can also override gains from here if you want:
                # {'kp_dist': 1.0},
                # {'kp_yaw': 2.0},
                # {'kd_yaw': 0.1},
                # {'max_linear_speed': 0.7},
                # {'max_angular_speed': 1.5},
                # {'pos_tolerance': 0.05},
            ]
        ),

        # --- JetRacer low-level node (unchanged) ---
        Node(
            package='jetracer',
            executable='jetracer',
            name='jetracer',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'port_name': '/dev/ttyACM0'},
                {'publish_odom_transform': False}
            ],
            remappings=[
                ('/odom', '/odom_raw')
            ]
        ),

        # --- EKF Localization Node ---
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='robot_pose_ekf',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},

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
            remappings=[
                ('/odom_raw', '/odom_raw'),
                ('imu', '/imu'),
                ('/odometry/filtered', '/odom_combined')
            ],
        ),

        # --- Static transform base_footprint -> IMU ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_imu',
            output='screen',
            arguments=[
                '0', '0', '0.02',   # x, y, z
                '0', '0', '0',      # roll, pitch, yaw
                'base_footprint',   # parent frame
                'base_umu_link'     # child frame (check spelling in your TF tree!)
            ]
        )
    ])

"""ROS side of the Isaac Sim JetRacer: the robot's own stack on simulated data.

    ros2 launch jetracer_sim sim.launch.py [agent:=sim] [grid:=true] [nav:=true]
                                           [nav_pose:=slam|wheels] [teleop:=true]

Runs orb_slam3.launch.py exactly as on the robot, but without the RealSense
and base drivers: Isaac Sim (isaac/run_isaac.py) supplies the images, and
cmd_vel_to_ackermann stands in for the base driver (drive commands, and the
true odometry as the "wheel odometry"). Depth arrives as 32FC1 metres, which
the SLAM node reads directly. The ground truth
is on /<agent>/ground_truth/odom. In the sim, /<agent>/odom (the speed Nav2
reads, and the "wheels" pose source) is the ground truth too.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    agent = LaunchConfiguration('agent').perform(context)
    share = get_package_share_directory('jetracer_sim')
    orb = get_package_share_directory('orb_slam3')
    flag = lambda name: LaunchConfiguration(name).perform(context)   # noqa: E731
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(orb, 'launch', 'orb_slam3.launch.py')),
            launch_arguments={
                'agent': agent,
                'vocab_file': os.path.expanduser('~/ORB_SLAM3/Vocabulary/ORBvoc.txt'),
                'settings_file': os.path.join(share, 'config', 'isaac_d455.yaml'),
                'ma_method': flag('ma_method'),
                'camera': 'false',
                'teleop_driver': 'false',
                'monitor': 'false',
                'occupancy_grid': flag('grid'),
                'nav': flag('nav'),
                'nav_pose': flag('nav_pose'),
                'teleop': flag('teleop'),
                'extra_params': os.path.join(share, 'config', 'sim_params.yaml'),
            }.items()),
        Node(package='jetracer_sim', executable='cmd_vel_to_ackermann', name='cmd_vel_to_ackermann',
             output='screen', parameters=[{'agent_name': agent}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('agent', default_value='sim'),
        DeclareLaunchArgument('ma_method', default_value='new'),
        DeclareLaunchArgument('grid', default_value='true'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('nav_pose', default_value='slam'),
        DeclareLaunchArgument('teleop', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])

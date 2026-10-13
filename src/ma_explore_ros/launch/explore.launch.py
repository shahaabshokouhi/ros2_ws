"""Autonomous exploration on one robot: the ma_explore planner (trained policy)
chooses this robot's goals and sends them to its Nav2.

    ros2 launch ma_explore_ros explore.launch.py [agent:=$AGENT_NAME] [agents:="robot1 robot2 robot3"]
        [policy:=FILE] [frames:=plain|prefixed] [device:=cpu|cuda] [start_enabled:=true]

Needs SLAM, the grid and Nav2 running on the robot (run_slam3.sh --nav, or
--explore, which starts this too). Usually started by run_slam3.sh --explore.

  agent    this robot (default $AGENT_NAME)
  agents   the team, this robot included (default: this robot only). The
           policy keeps its training team size; robots that are not there
           count as never merged.
  policy   default: <workspace>/src/ma_explore_ros/policy/policy.pt (copy the
           trained network there; no rebuild needed), else the installed copy
  frames   plain: map_nav, odom, ... (run_slam3.sh --nav); prefixed:
           <agent>/map_nav, ... (--ma-frames, several robots on one ROS domain)
  start_enabled  false: wait for /ma_explore/<agent>/enable (std_msgs/Bool true)
Decisions, logs and tracking-loss events: ~/result/ma_explore_robot/<date>_<time>_<agent>/.
"""
import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_policy():
    share = get_package_share_directory('ma_explore_ros')
    ws = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))      # <ws>/install/ma_explore_ros/share/ma_explore_ros
    for p in (os.path.join(ws, 'src', 'ma_explore_ros', 'policy', 'policy.pt'), os.path.join(share, 'policy', 'policy.pt')):
        if os.path.exists(p):
            return p
    return os.path.join(ws, 'src', 'ma_explore_ros', 'policy', 'policy.pt')


def setup(context):
    get = lambda n: LaunchConfiguration(n).perform(context).strip()    # noqa: E731
    agent = get('agent') or os.environ.get('AGENT_NAME', '')
    if not agent:
        raise RuntimeError('agent:= not given and AGENT_NAME not set')
    agents = get('agents') or agent
    if agent not in agents.split():
        agents = agent + ' ' + agents
    policy = os.path.expanduser(get('policy') or default_policy())
    if not os.path.exists(policy) and policy.endswith('.pt'):
        raise RuntimeError(f'no policy file {policy}: copy the trained robot_policy.pt there (see policy/README.md)')
    frames = get('frames')
    if frames not in ('plain', 'prefixed'):
        raise RuntimeError("frames must be plain or prefixed")
    out = os.path.join(os.path.expanduser(get('out')), datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + f'_{agent}')
    return [Node(
        package='ma_explore_ros', executable='planner', name=f'ma_explore_planner_{agent}', output='screen',
        parameters=[{'agent': agent, 'agents': agents, 'policy': policy, 'device': get('device'),
                     'frame_prefix': 'agent' if frames == 'prefixed' else 'none', 'out': out,
                     'start_enabled': get('start_enabled').lower() in ('true', '1', 'yes')}],
        # torch.compile needs a C++ toolchain and takes long to warm up on the robot: plain kernels
        additional_env={'MA_EXPLORE_NO_COMPILE': '1', 'OMP_NUM_THREADS': '2'},
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('agent', default_value=''),
        DeclareLaunchArgument('agents', default_value=''),
        DeclareLaunchArgument('policy', default_value=''),
        DeclareLaunchArgument('frames', default_value='plain'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('start_enabled', default_value='true'),
        DeclareLaunchArgument('out', default_value='~/result/ma_explore_robot'),
        OpaqueFunction(function=setup),
    ])

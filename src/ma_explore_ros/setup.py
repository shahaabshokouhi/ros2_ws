from glob import glob

from setuptools import setup

package = 'ma_explore_ros'
setup(
    name=package,
    version='0.1.0',
    packages=[package],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package]),
        ('share/' + package, ['package.xml', 'README.md']),
        ('share/' + package + '/launch', glob('launch/*.py')),
        ('share/' + package + '/policy', ['policy/README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shahab',
    description='Autonomous exploration with the trained ma_explore policy',
    license='Proprietary',
    entry_points={'console_scripts': [
        'planner = ma_explore_ros.ros_planner:main',
        'robot_view = ma_explore_ros.robot_view:main',
        'policy_info = ma_explore_ros.policy_info:main',
    ]},
)

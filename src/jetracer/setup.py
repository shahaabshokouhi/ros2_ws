from setuptools import setup

package_name = 'jetracer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/joy.launch.py',
            'launch/pid_controller.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/waypoints.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='JetRacer hybrid package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_joy = scripts.teleop_joy:main',
            'pid_controlller = scripts.pid_controller:main',
        ],
    },
)

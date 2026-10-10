from glob import glob

from setuptools import setup

package_name = 'jetracer_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/isaac', glob('isaac/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shahab Shokouhi',
    maintainer_email='shokohishahab@gmail.com',
    description='Isaac Sim stand-in for one JetRacer',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cmd_vel_to_ackermann = jetracer_sim.cmd_vel_to_ackermann:main',
        ],
    },
)

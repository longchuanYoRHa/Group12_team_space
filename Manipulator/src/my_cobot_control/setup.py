from setuptools import find_packages, setup
import sys
import os
from glob import glob

package_name = 'my_cobot_control'

########################
# Use venv Python only on dev machine (auto-detect by path existence).
# On the real robot Pi, pymycobot is installed system-wide so no venv needed.
venv_python = '/home/student42/mycobot_ws/venv_mycobot/bin/python3'
use_venv = os.path.exists(venv_python)
########################

_extra_options = {}
if use_venv:
    _extra_options['options'] = {
        'build_scripts': {
            'executable': venv_python,
        },
    }

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student42',
    maintainer_email='zhikun.peng929@outlook.com',
    description='MyCobot 280 Pi ROS2 arm controller with adaptive gripper',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_and_place_rviz = my_cobot_control.pick_and_place_rviz:main',
            'mycobot_controller = my_cobot_control.mycobot_controller:main',
            'mycobot_controller_tf2 = my_cobot_control.mycobot_controller_tf2:main',
        ],
    },
    **_extra_options,
)

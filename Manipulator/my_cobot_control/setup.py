from setuptools import find_packages, setup
import sys
import os
from glob import glob

package_name = 'my_cobot_control'

########################
# Python using a virtual environment (delete this paragraph on real robot)
# 获取虚拟环境路径 （机械臂上删除此段）
venv_python = '/home/student42/mycobot_ws/venv_mycobot/bin/python3'
########################

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_to_default = my_cobot_control.move_to_default:main',
            'pick_and_place = my_cobot_control.pick_and_place:main',
            'move = my_cobot_control.move:main',
            'pick_and_place_rviz = my_cobot_control.pick_and_place_rviz:main',
            'pick_and_place_with_feedback = my_cobot_control.pick_and_place_with_feedback:main',
        ],
    },
    ##################################################
    # Python using a virtual environment (delete this paragraph on real robot)
    # 使用虚拟环境的 Python （机械臂上删除此段）
    options={
        'build_scripts': {
            'executable': venv_python,
        },
    },
    ##################################################
)

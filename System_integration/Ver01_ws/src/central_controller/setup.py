from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'central_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*launch.[pxy][yma]*')),
         (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
         (os.path.join('share', package_name, 'config'),
         glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student21',
    maintainer_email='2674312287@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_manager = central_controller.task_manager:main',
            'mock_arm_node = central_controller.mock_arm_node:main'
        ],
    },
)

from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'real_Nav_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*launch.[pxy][yma]*')),

        # Install config files
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
            'task_manager_node = real_Nav_test.task_manager_node:main',
            'approach_from_boxes_test_node = real_Nav_test.approach_from_boxes_test_node:main',
            'example_vision_publisher = real_Nav_test.example_vision_publisher:main',
            'task_manager_node_v2 = real_Nav_test.task_manager_node_v2:main',
            'mock_arm_node = real_Nav_test.mock_arm_node:main',
        ],
    },
)

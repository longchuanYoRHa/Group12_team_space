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
         glob('config/*.yaml') + glob('config/*.xml')),
         (os.path.join('share', package_name, 'maps'),
         glob('maps/*')),
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
            'mock_arm_node = central_controller.mock_arm_node:main',
            'task_manager_v2 = central_controller.task_manager_node_v2:main',
            'task_manager_v3 = central_controller.task_manager_node_v3:main',
            'task_manager_v4 = central_controller.task_manager_node_v4:main',
            'detect_objects_in_pgm_map = central_controller.detect_objects_in_pgm_map:main',
            'module_test_docking = central_controller.module_test_docking:main',
            'module_test_box_mapping = central_controller.module_test_box_mapping:main',
            'module_test_explore_approach = central_controller.module_test_explore_approach:main',
            'mock_target_pick_red_publisher = central_controller.mock_target_pick_red_publisher:main',
            'mock_target_place_red_publisher = central_controller.mock_target_place_red_publisher:main',
        ],
    },
)

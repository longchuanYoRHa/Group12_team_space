#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('my_cobot_control')
    
    # RViz 配置文件
    rviz_config = os.path.join(pkg_share, 'rviz', 'mycobot_tf2.rviz')
    
    # 包含主 launch 文件
    mycobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_cobot_control'),
                'launch',
                'mycobot_with_tf2.launch.py'
            ])
        ])
    )
    
    # RViz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )
    
    return LaunchDescription([
        mycobot_launch,
        rviz_node,
    ])
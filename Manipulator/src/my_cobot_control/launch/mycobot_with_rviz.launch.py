#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('my_cobot_control')
    
    # RViz configuration file path
    rviz_config = os.path.join(pkg_share, 'rviz', 'mycobot_tf2.rviz')
    
    # Include main launch file
    mycobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_cobot_control'),
                'launch',
                'mycobot_with_tf2.launch.py'
            ])
        ])
    )
    
    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )

    # The workspace limits marker is published as a static topic for visualization in RViz. 
    # It defines a box representing the arm's reachable workspace.
    workspace_marker_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '-r', '1',
            '/arm/workspace_limits', 'visualization_msgs/msg/Marker',
            '{"header": {"frame_id": "g_base"}, "ns": "workspace", "id": 0, "type": 1, "action": 0, '
            '"pose": {"position": {"x": 0.195, "y": -0.0525, "z": 0.035}, "orientation": {"w": 1.0}}, '
            '"scale": {"x": 0.170, "y": 0.215, "z": 0.100}, '
            '"color": {"r": 0.0, "g": 1.0, "b": 0.0, "a": 0.25}}'
        ],
        output='log'
    )
    
    return LaunchDescription([
        mycobot_launch,
        rviz_node,
        workspace_marker_pub
    ])
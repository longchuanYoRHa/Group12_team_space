#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    rplidar_pkg_dir = FindPackageShare('rplidar_ros')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    explore_lite_launch = PathJoinSubstitution(
        [FindPackageShare('explore_lite'), 'launch', 'explore.launch.py']
    )

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 1. Launch RPLidar (without rviz)
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rplidar_pkg_dir, 'launch', 'rplidar_a2m12_launch.py'])
        )
    )

    # 2. Wait for /scan topic to be available
    # Using a script to wait for topic availability
    wait_for_scan_cmd = ExecuteProcess(
        cmd=['bash', '-c', 
             'until ros2 topic list | grep -q "/scan"; do echo "Waiting for /scan topic..."; sleep 1; done; echo "/scan topic is available"'],
        output='screen'
    )

    # 3. Static transform from base_link to laser
    # Lidar position: X+ 240.7mm (0.2407m), Z- 69.18mm (-0.06918m), upside down (180 deg rotation around X axis)
    # For upside down: roll=π (3.14159 radians), pitch=0, yaw=0
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '0.2407', '0.0', '-0.06918',  # x, y, z in meters
            '3.14159', '0', '0',  # roll, pitch, yaw in radians (roll=π for 180 deg rotation around X axis)
            'base_link', 'laser'
        ],
        output='screen'
    )

    # 4. Launch SLAM toolbox node with nav2_bringup default config and override specific parameters
    # nav2_bringup uses mapper_params_online_sync.yaml from slam_toolbox package as default
    # when slam_toolbox params are not in nav2_params.yaml
    slam_toolbox_params_file = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'config', 'mapper_params_online_sync.yaml']
    )
    
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_toolbox_params_file],  # Load nav2_bringup default config
        arguments=[
            '--ros-args',
            '-p', 'use_sim_time:=false',
            '-p', 'base_frame:=base_link',
            '-p', 'map_frame:=map',
            '-p', 'odom_frame:=odom',
            '-p', 'provide_odom_frame:=true',
            '-p', 'use_odom:=false',
            '-p', 'scan_topic:=/scan',
            '--log-level', 'slam_toolbox:=debug'
        ]
    )

    # Launch nav2_bringup navigation_launch.py
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # 5. Configure and activate slam_toolbox lifecycle
    # First configure
    configure_slam_cmd = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
        output='screen'
    )

    # Then activate (after configure completes)
    activate_slam_cmd = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
        output='screen'
    )

    # 6. Launch explore.launch.py
    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([explore_lite_launch]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Sequence the actions:
    # 1. Start lidar
    # 2. Wait for /scan (delayed after lidar starts)
    # 3. Publish static transform (after scan is available)
    # 4. Launch SLAM (after transform is published)
    # 5. Configure SLAM (after SLAM launch)
    # 6. Activate SLAM (after configure)
    # 7. Launch explore (after SLAM is activated)

    # Wait for scan after lidar starts (with delay)
    wait_for_scan_delayed = TimerAction(
        period=3.0,  # Give lidar time to start
        actions=[wait_for_scan_cmd]
    )

    # Publish static transform after scan is available
    static_tf_delayed = TimerAction(
        period=5.0,  # Wait a bit more to ensure scan is publishing
        actions=[static_tf_base_to_laser]
    )

    # Launch SLAM and Nav2 after transform is published
    slam_toolbox_delayed = TimerAction(
        period=7.0,  # Wait for transform to be established
        actions=[slam_toolbox_node]
    )

    nav2_launch_delayed = TimerAction(
        period=7.0,  # Launch Nav2 at the same time as SLAM
        actions=[nav2_launch]
    )

    # Configure SLAM after it's launched (with delay)
    configure_slam_delayed = TimerAction(
        period=10.0,  # Give SLAM time to start
        actions=[configure_slam_cmd]
    )

    # Activate SLAM after configure (with delay)
    activate_slam_delayed = TimerAction(
        period=12.0,  # Give configure time to complete
        actions=[activate_slam_cmd]
    )

    # Launch explore after SLAM is activated
    explore_launch_delayed = TimerAction(
        period=15.0,  # Give activate time to complete
        actions=[explore_launch]
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        
        # Start lidar first
        rplidar_launch,
        
        # Wait for scan topic
        wait_for_scan_delayed,
        
        # Publish static transform
        static_tf_delayed,
        
        # Launch SLAM toolbox
        slam_toolbox_delayed,
        
        # Launch Nav2
        nav2_launch_delayed,
        
        # Configure SLAM lifecycle
        configure_slam_delayed,
        
        # Activate SLAM lifecycle
        activate_slam_delayed,
        
        # Launch explore
        explore_launch_delayed,
    ])


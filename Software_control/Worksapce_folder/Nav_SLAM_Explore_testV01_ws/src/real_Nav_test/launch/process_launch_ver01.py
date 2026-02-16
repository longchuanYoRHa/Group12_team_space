#!/usr/bin/env python3
"""
Process Launch File for Explore-Pick-Stow-SearchBin-Place System
Integrates chassis system (from real_nav_test_launch.py) with task manager state machine
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Check if lidar is connected
    lidar_connected = False
    common_serial_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
    
    for port in common_serial_ports:
        if os.path.exists(port):
            lidar_connected = True
            print(f"[INFO] Detected LiDAR device: {port}")
            break
    
    if not lidar_connected:
        print("[WARN] No LiDAR detected, will skip LiDAR launch (simulation mode)")
    
    # Get package directories
    rplidar_pkg_dir = FindPackageShare('rplidar_ros')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    real_nav_test_pkg_dir = FindPackageShare('real_Nav_test')
    explore_lite_launch = PathJoinSubstitution(
        [FindPackageShare('explore_lite'), 'launch', 'explore.launch.py']
    )

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Create condition variable
    lidar_connected_str = 'true' if lidar_connected else 'false'
    lidar_connected_config = LaunchConfiguration('lidar_connected', default=lidar_connected_str)

    # 1. Launch RPLidar (without rviz) - only when lidar is connected
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rplidar_pkg_dir, 'launch', 'rplidar_a2m12_launch.py'])
        ),
        condition=IfCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"]))
    )

    # Wait for scan topic
    wait_for_scan_cmd = ExecuteProcess(
        cmd=['bash', '-c', 
             'until ros2 topic list | grep -q "/scan"; do echo "Waiting for /scan topic..."; sleep 1; done; echo "/scan topic is available"'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"]))
    )

    # 3. Static transform from base_link to laser (only when lidar is connected)
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '0.2407', '0.0', '-0.06918',  # x, y, z in meters
            '3.14159', '0', '0',  # roll, pitch, yaw in radians
            'base_link', 'laser'
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"]))
    )

    # Laser filter node
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[PathJoinSubstitution([real_nav_test_pkg_dir, 'config', 'scan_filter.yaml'])],
    )

    # 4. Launch SLAM toolbox node
    slam_toolbox_params_file = PathJoinSubstitution(
        [real_nav_test_pkg_dir, 'config', 'mapper_params_online_async.yaml']
    )
    
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_toolbox_params_file],
        output='screen',
    )

    # Launch nav2_bringup navigation_launch.py
    nav2_params_file = PathJoinSubstitution([real_nav_test_pkg_dir, 'config', 'nav2_params.yaml'])

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items()
    )

    # 5. Configure and activate slam_toolbox lifecycle
    configure_slam_cmd = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
        output='screen'
    )

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

    # 7. Launch Task Manager State Machine Node
    task_manager_node = Node(
        package='real_Nav_test',
        executable='task_manager_node',
        name='task_manager',
        output='screen',
        parameters=[{
            'pregrasp_distance': 0.5,  # meters
            'preplace_distance': 0.6,  # meters
            'stow_pose_x': 0.3,
            'stow_pose_y': 0.0,
            'stow_pose_z': 0.2,
        }]
    )

    # Launch rviz with custom config
    rviz_config_file = PathJoinSubstitution(
        [real_nav_test_pkg_dir, 'rviz', 'nav2_default_view.rviz']
    )
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'rviz_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config_file': rviz_config_file,
        }.items()
    )

    # Delayed actions for proper sequencing
    wait_for_scan_delayed = TimerAction(
        period=3.0,
        actions=[wait_for_scan_cmd],
    )

    static_tf_delayed = TimerAction(
        period=5.0,
        actions=[static_tf_base_to_laser],
    )

    laser_filter_delayed = TimerAction(
        period=5.0,
        actions=[laser_filter_node]
    )

    slam_toolbox_delayed = TimerAction(
        period=7.0,
        actions=[slam_toolbox_node]
    )

    nav2_launch_delayed = TimerAction(
        period=7.0,
        actions=[nav2_launch]
    )

    configure_slam_delayed = TimerAction(
        period=10.0,
        actions=[configure_slam_cmd]
    )

    activate_slam_delayed = TimerAction(
        period=12.0,
        actions=[activate_slam_cmd]
    )

    explore_launch_delayed = TimerAction(
        period=15.0,
        actions=[explore_launch]
    )

    # Task manager starts after Nav2 and explore are ready
    task_manager_delayed = TimerAction(
        period=18.0,
        actions=[task_manager_node]
    )

    rviz_launch_delayed = TimerAction(
        period=20.0,
        actions=[rviz_launch]
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        
        DeclareLaunchArgument(
            'lidar_connected',
            default_value=lidar_connected_str,
            description='Whether lidar is connected (auto-detected)'
        ),
        
        # Log detection results
        LogInfo(
            msg=['LiDAR connection status: Connected, will start LiDAR node'],
            condition=IfCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"]))
        ),
        LogInfo(
            msg='LiDAR not connected, skipping LiDAR launch (simulation mode)',
            condition=UnlessCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"]))
        ),
        
        # Start lidar first (only when connected)
        rplidar_launch,
        
        # Wait for scan topic
        wait_for_scan_delayed,
        
        # Publish static transform
        static_tf_delayed,
        
        # Launch laser filter
        laser_filter_delayed,
        
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
        
        # Launch task manager (after system is ready)
        task_manager_delayed,
        
        # Launch rviz
        rviz_launch_delayed,
    ])

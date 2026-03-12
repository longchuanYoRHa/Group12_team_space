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
             'timeout=60; elapsed=0; '
             'until ros2 topic list | grep -q "^/scan$"; do '
             '  echo "Waiting for /scan topic... ($elapsed/$timeout seconds)"; '
             '  sleep 1; elapsed=$((elapsed+1)); '
             '  if [ $elapsed -ge $timeout ]; then '
             '    echo "ERROR: /scan topic not available after $timeout seconds"; exit 1; '
             '  fi; '
             'done; '
             'echo "/scan topic is available"'],
        output='screen'
    )

    # 3. Static transform from base_link to laser (only when lidar is connected)
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '0.1758', '0.0', '-0.10553',  # x, y, z in meters
            '3.14159', '0', '3.14159',  # roll, pitch, yaw in radians
            'base_link', 'laser'
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"]))
    )

    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=[
            '0.14813', '0.05957', '0.027',  # x, y, z in meters
            '0.0', '0.0', '0.0',  # roll, pitch, yaw in radians
            'base_link', 'depth_camera'
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

    # Wait for SLAM lifecycle node to be available
    wait_for_slam_node_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             'timeout=30; elapsed=0; '
             'until ros2 node list | grep -q "slam_toolbox"; do '
             '  echo "Waiting for slam_toolbox node... ($elapsed/$timeout seconds)"; '
             '  sleep 1; elapsed=$((elapsed+1)); '
             '  if [ $elapsed -ge $timeout ]; then '
             '    echo "ERROR: slam_toolbox node not available after $timeout seconds"; exit 1; '
             '  fi; '
             'done; '
             'echo "slam_toolbox node is available"'],
        output='screen'
    )

    # 5. Configure and activate slam_toolbox lifecycle
    configure_slam_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             'ros2 lifecycle set /slam_toolbox configure && '
             'echo "SLAM configured successfully"'],
        output='screen'
    )

    # Wait for SLAM to be configured before activating
    wait_for_slam_configured_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             r'timeout=10; elapsed=0; '
             r'until ros2 lifecycle get /slam_toolbox | grep -q "active\|inactive"; do '
             '  echo "Waiting for SLAM to be configured... ($elapsed/$timeout seconds)"; '
             '  sleep 0.5; elapsed=$((elapsed+1)); '
             r'  if [ $elapsed -ge $timeout ]; then '
             r'    echo "ERROR: SLAM configuration timeout"; exit 1; '
             r'  fi; '
             r'done; '
             r'echo "SLAM is configured"'],
        output='screen'
    )

    activate_slam_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             'ros2 lifecycle set /slam_toolbox activate && '
             'echo "SLAM activated successfully"'],
        output='screen'
    )

    # Wait for /map topic to be available (after SLAM is activated)
    wait_for_map_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             'timeout=30; elapsed=0; '
             'until ros2 topic list | grep -q "^/map$"; do '
             '  echo "Waiting for /map topic... ($elapsed/$timeout seconds)"; '
             '  sleep 1; elapsed=$((elapsed+1)); '
             '  if [ $elapsed -ge $timeout ]; then '
             '    echo "ERROR: /map topic not available after $timeout seconds"; exit 1; '
             '  fi; '
             'done; '
             'echo "/map topic is available"'],
        output='screen'
    )

    # Wait for navigate_to_pose action server to be available
    wait_for_nav_action_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             'timeout=60; elapsed=0; '
             'until ros2 action list | grep -q "navigate_to_pose"; do '
             '  echo "Waiting for navigate_to_pose action server... ($elapsed/$timeout seconds)"; '
             '  sleep 1; elapsed=$((elapsed+1)); '
             '  if [ $elapsed -ge $timeout ]; then '
             '    echo "ERROR: navigate_to_pose action server not available after $timeout seconds"; exit 1; '
             '  fi; '
             'done; '
             'echo "navigate_to_pose action server is available"'],
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
            'pregrasp_distance': 0.5,
            'preplace_distance': 0.6,
            'camera_frame_id': 'camera_depth_optical_frame',
        }]
    )

    # Launch rviz with custom config
    # rviz_config_file = PathJoinSubstitution(
    #     [real_nav_test_pkg_dir, 'rviz', 'nav2_default_view.rviz']
    # )
    # rviz_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([nav2_bringup_dir, 'launch', 'rviz_launch.py'])
    #     ),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'rviz_config_file': rviz_config_file,
    #     }.items()
    # )

    
    # 1. Wait for /scan topic available, then start static transform and laser filter
    static_tf_after_scan = TimerAction(
        period=0.1,  # short delay, ensure the wait command completes
        actions=[static_tf_base_to_laser]
    )
    
    laser_filter_after_scan = TimerAction(
        period=0.1,
        actions=[laser_filter_node]
    )
    
    wait_for_scan_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_scan_cmd,
            on_exit=[
                static_tf_after_scan,static_tf_base_to_camera,
                laser_filter_after_scan,
            ]
        )
    )

    # 2. After /scan available, delay start SLAM (give transform and filter time to establish)
    slam_after_scan = TimerAction(
        period=2.0,  # give transform and filter time to establish
        actions=[slam_toolbox_node]
    )
    
    slam_start_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_scan_cmd,
            on_exit=[slam_after_scan]
        )
    )

    # 3. After SLAM node starts, wait for node available, then configure
    # SLAM starts after 2 seconds when /scan is available, so the waiting node should be after 5 seconds (2 seconds start + 3 seconds wait)
    wait_slam_node_after_start = TimerAction(
        period=5.0,  # wait for SLAM node after 5 seconds (2 seconds start + 3 seconds wait)
        actions=[wait_for_slam_node_cmd]
    )
    
    slam_node_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_scan_cmd,  # after scan available, delay wait for SLAM node
            on_exit=[wait_slam_node_after_start]
        )
    )

    # 4. After SLAM node available, configure SLAM
    wait_for_slam_node_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_slam_node_cmd,
            on_exit=[configure_slam_cmd]
        )
    )

    # 5. After configuring SLAM, wait for configuration to complete, then activate
    configure_slam_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=configure_slam_cmd,
            on_exit=[wait_for_slam_configured_cmd]
        )
    )

    # 6. After configuring SLAM, activate SLAM
    wait_for_slam_configured_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_slam_configured_cmd,
            on_exit=[activate_slam_cmd]
        )
    )

    # 7. After SLAM activated, wait for /map topic available, then start Nav2
    activate_slam_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=activate_slam_cmd,
            on_exit=[wait_for_map_cmd]
        )
    )

    # 8. After /map topic available, start Nav2
    nav2_after_map = TimerAction(
        period=0.1,  # short delay
        actions=[nav2_launch]
    )
    
    wait_for_map_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map_cmd,
            on_exit=[nav2_after_map]
        )
    )

    # 9. After Nav2 starts, wait for navigate_to_pose action available, then start explore
    # Note: nav2_launch is IncludeLaunchDescription, will not trigger OnProcessExit
    # So after /map available, delay wait for action server
    wait_nav_action_after_map = TimerAction(
        period=8.0,  # give Nav2 time to start (after /map available in 8 seconds)
        actions=[wait_for_nav_action_cmd]
    )
    
    nav2_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map_cmd,  # after map available, delay wait for action
            on_exit=[wait_nav_action_after_map]
        )
    )

    # 10. After navigate_to_pose action available, start explore
    explore_after_nav_action = TimerAction(
        period=0.1,  # short delay
        actions=[explore_launch]
    )

    # 11. After explore starts, start task manager and rviz
    # Note: explore_launch is IncludeLaunchDescription, will not trigger OnProcessExit
    # So after navigate_to_pose available, delay start task manager and rviz
    task_manager_after_nav_action = TimerAction(
        period=3.0,  # give explore time to start
        actions=[task_manager_node]
    )

    # After navigate_to_pose action available, start explore and task manager
    explore_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_nav_action_cmd,
            on_exit=[
                explore_after_nav_action,
                task_manager_after_nav_action,
            ]
        )
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
        
        # start in strict topic dependency order:
        # 1. start lidar (if connected)
        rplidar_launch,
        
        # 2. wait for /scan topic available (both simulation and actual)
        wait_for_scan_cmd,
        
        # 3-11. start subsequent nodes through event handler chain
        wait_for_scan_handler,      # after /scan available, start static_tf and laser_filter
        slam_start_handler,          # after /scan available, delay start SLAM
        slam_node_handler,           # after /scan available, delay wait for SLAM node
        wait_for_slam_node_handler,  # after SLAM node available, configure SLAM
        configure_slam_handler,      # after configuring, wait for configuration to complete
        wait_for_slam_configured_handler,  # after configuring, activate SLAM
        activate_slam_handler,       # after activating, wait for /map
        wait_for_map_handler,        # after /map available, start Nav2
        nav2_handler,                # after /map available, delay wait for navigate_to_pose action
        explore_handler,              # after action available, start explore + task manager
    ])

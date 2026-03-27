import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
    LogInfo,
)
from launch.event_handlers import OnProcessExit
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
    controller_pkg_dir = FindPackageShare('central_controller')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    camera_frame_id = LaunchConfiguration('camera_frame_id', default='camera_link')
    target_bin_color = LaunchConfiguration('target_bin_color', default='')
    docking_stop_distance_m = LaunchConfiguration('docking_stop_distance_m', default='0.20')

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

    # Static transform from base_link to laser (only when lidar is connected)
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '0.1758', '0.0', '-0.10553',
            '3.14159', '0', '3.14159',
            'base_link', 'laser'
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"]))
    )

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[PathJoinSubstitution([controller_pkg_dir, 'config', 'scan_filter.yaml'])],
    )

    # Launch SLAM toolbox node
    slam_toolbox_params_file = PathJoinSubstitution(
        [controller_pkg_dir, 'config', 'mapper_params_online_async.yaml']
    )
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_toolbox_params_file],
        output='screen',
    )

    # Launch nav2_bringup navigation_launch.py
    nav2_params_file = PathJoinSubstitution([controller_pkg_dir, 'config', 'nav2_params_radius.yaml'])
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items()
    )

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

    configure_slam_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             'ros2 lifecycle set /slam_toolbox configure && '
             'echo "SLAM configured successfully"'],
        output='screen'
    )

    wait_for_slam_configured_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             'timeout=10; elapsed=0; '
             'until ros2 lifecycle get /slam_toolbox | grep -q "active\\|inactive"; do '
             '  echo "Waiting for SLAM to be configured... ($elapsed/$timeout seconds)"; '
             '  sleep 0.5; elapsed=$((elapsed+1)); '
             '  if [ $elapsed -ge $timeout ]; then '
             '    echo "ERROR: SLAM configuration timeout"; exit 1; '
             '  fi; '
             'done; '
             'echo "SLAM is configured"'],
        output='screen'
    )

    activate_slam_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             'ros2 lifecycle set /slam_toolbox activate && '
             'echo "SLAM activated successfully"'],
        output='screen'
    )

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

    # Module03: initial mapping + cached-coordinate docking test
    module_test_box_mapping_node = Node(
        package='central_controller',
        executable='module_test_box_mapping',
        name='module_test_box_mapping',
        output='screen',
        parameters=[{
            'camera_frame_id': camera_frame_id,
            'dock_type': 'simple_non_charging_dock',
            'docking_stop_distance_m': docking_stop_distance_m,
            'target_bin_color': target_bin_color,
            'use_sim_time': use_sim_time,
        }]
    )

    static_tf_after_scan = TimerAction(
        period=0.1,
        actions=[static_tf_base_to_laser]
    )

    laser_filter_after_scan = TimerAction(
        period=0.1,
        actions=[laser_filter_node]
    )

    wait_for_scan_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_scan_cmd,
            on_exit=[static_tf_after_scan, laser_filter_after_scan]
        )
    )

    slam_after_scan = TimerAction(
        period=2.0,
        actions=[slam_toolbox_node]
    )

    slam_start_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_scan_cmd,
            on_exit=[slam_after_scan]
        )
    )

    wait_slam_node_after_start = TimerAction(
        period=3.0,
        actions=[wait_for_slam_node_cmd]
    )

    slam_node_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_scan_cmd,
            on_exit=[wait_slam_node_after_start]
        )
    )

    wait_for_slam_node_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_slam_node_cmd,
            on_exit=[configure_slam_cmd]
        )
    )

    configure_slam_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=configure_slam_cmd,
            on_exit=[wait_for_slam_configured_cmd]
        )
    )

    wait_for_slam_configured_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_slam_configured_cmd,
            on_exit=[activate_slam_cmd]
        )
    )

    activate_slam_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=activate_slam_cmd,
            on_exit=[wait_for_map_cmd]
        )
    )

    reset_odometry_cmd = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            'timeout=15; elapsed=0; '
            'until ros2 service list | grep -q "^/reset_odometry$"; do '
            '  echo "Waiting for /reset_odometry service... ($elapsed/$timeout seconds)"; '
            '  sleep 1; elapsed=$((elapsed+1)); '
            '  if [ $elapsed -ge $timeout ]; then '
            '    echo "WARN: /reset_odometry service not available after $timeout seconds; skipping reset"; exit 0; '
            '  fi; '
            'done; '
            'echo "Calling /reset_odometry ..."; '
            'ros2 service call /reset_odometry std_srvs/srv/Trigger "{}" && '
            'echo "/reset_odometry call done"',
        ],
        output='screen',
        condition=UnlessCondition(PythonExpression(["'", use_sim_time, "' == 'true'"]))
    )

    nav2_after_map = TimerAction(
        period=0.1,
        actions=[nav2_launch],
        condition=UnlessCondition(PythonExpression(["'", use_sim_time, "' == 'true'"]))
    )

    nav2_after_map_sim = TimerAction(
        period=0.1,
        actions=[nav2_launch],
        condition=IfCondition(PythonExpression(["'", use_sim_time, "' == 'true'"]))
    )

    wait_for_map_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map_cmd,
            on_exit=[reset_odometry_cmd, nav2_after_map_sim]
        )
    )

    reset_odometry_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=reset_odometry_cmd,
            on_exit=[nav2_after_map]
        )
    )

    module_test_after_nav_action = TimerAction(
        period=3.0,
        actions=[module_test_box_mapping_node]
    )

    wait_nav_action_after_map = TimerAction(
        period=8.0,
        actions=[wait_for_nav_action_cmd]
    )

    nav2_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map_cmd,
            on_exit=[wait_nav_action_after_map]
        )
    )

    wait_for_nav_action_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_nav_action_cmd,
            on_exit=[module_test_after_nav_action]
        )
    )

    return LaunchDescription([
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
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='camera_link',
            description='TF frame id for vision 3D points (e.g. camera_link or D435i_camera_link)'
        ),
        DeclareLaunchArgument(
            'target_bin_color',
            default_value='',
            description='Preferred cached bin color for module03 docking test'
        ),
        DeclareLaunchArgument(
            'docking_stop_distance_m',
            default_value='0.20',
            description='Stop distance from cached bin target in meters'
        ),
        LogInfo(
            msg=['LiDAR connection status: Connected, will start LiDAR node'],
            condition=IfCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"]))
        ),
        LogInfo(
            msg='LiDAR not connected, skipping LiDAR launch (simulation mode)',
            condition=UnlessCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"]))
        ),
        rplidar_launch,
        wait_for_scan_cmd,
        wait_for_scan_handler,
        slam_start_handler,
        slam_node_handler,
        wait_for_slam_node_handler,
        configure_slam_handler,
        wait_for_slam_configured_handler,
        activate_slam_handler,
        wait_for_map_handler,
        reset_odometry_handler,
        nav2_handler,
        wait_for_nav_action_handler,
    ])

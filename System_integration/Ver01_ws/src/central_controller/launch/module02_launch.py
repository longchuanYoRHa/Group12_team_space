import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # LiDAR presence (same heuristic as starter_launch)
    lidar_connected = False
    common_serial_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
    for port in common_serial_ports:
        if os.path.exists(port):
            lidar_connected = True
            print(f"[INFO] Detected LiDAR device: {port}")
            break
    if not lidar_connected:
        print("[WARN] No LiDAR detected, will skip LiDAR launch (simulation mode)")

    use_sim_time_str = 'true' if not lidar_connected else 'false'
    use_sim_time_subst = TextSubstitution(text=use_sim_time_str)

    rplidar_pkg_dir = FindPackageShare('rplidar_ros')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    controller_pkg_dir = FindPackageShare('central_controller')
    custom_explore_params = PathJoinSubstitution(
        [FindPackageShare('custom_explore'), 'config', 'params.yaml']
    )

    lidar_connected_str = 'true' if lidar_connected else 'false'
    lidar_connected_config = LaunchConfiguration('lidar_connected', default=lidar_connected_str)

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rplidar_pkg_dir, 'launch', 'rplidar_a2m12_launch.py'])
        ),
        condition=IfCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"])),
    )

    wait_for_scan_cmd = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            'timeout=120; elapsed=0; '
            'until ros2 topic list | grep -q "^/scan$"; do '
            '  echo "Waiting for /scan topic... ($elapsed/$timeout x0.1s)"; '
            '  sleep 0.1; elapsed=$((elapsed+1)); '
            '  if [ $elapsed -ge $((timeout * 10)) ]; then '
            '    echo "ERROR: /scan topic not available"; exit 1; '
            '  fi; '
            'done; '
            'echo "/scan topic is available"',
        ],
        output='screen',
    )

    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '0.1758', '0.0', '-0.10553',
            '3.14159', '0', '3.14159',
            'base_link', 'laser',
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"])),
    )

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[PathJoinSubstitution([controller_pkg_dir, 'config', 'scan_filter.yaml'])],
    )

    slam_toolbox_params_file = PathJoinSubstitution(
        [controller_pkg_dir, 'config', 'mapper_params_online_async.yaml']
    )
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_toolbox_params_file, {'use_sim_time': not lidar_connected}],
        output='screen',
    )

    slam_configure_activate_cmd = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            'timeout=120; elapsed=0; '
            'until ros2 lifecycle get /slam_toolbox 2>/dev/null | grep -q "unconfigured"; do '
            '  sleep 0.1; elapsed=$((elapsed+1)); '
            '  if [ $elapsed -ge $((timeout * 10)) ]; then '
            '    echo "ERROR: slam_toolbox not unconfigured in time"; exit 1; '
            '  fi; '
            'done; '
            'ros2 lifecycle set /slam_toolbox configure && '
            'ros2 lifecycle set /slam_toolbox activate && '
            'echo "SLAM configure+activate done"',
        ],
        output='screen',
    )

    nav2_params_file = PathJoinSubstitution(
        [controller_pkg_dir, 'config', 'nav2_params_smac2d_rpp.yaml']
    )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time_subst,
            'params_file': nav2_params_file,
        }.items(),
    )

    wait_for_map_cmd = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            'timeout=120; elapsed=0; '
            'until ros2 topic list | grep -q "^/map$"; do '
            '  echo "Waiting for /map topic... ($elapsed/$timeout x0.1s)"; '
            '  sleep 0.1; elapsed=$((elapsed+1)); '
            '  if [ $elapsed -ge $((timeout * 10)) ]; then '
            '    echo "ERROR: /map topic not available"; exit 1; '
            '  fi; '
            'done; '
            'echo "/map topic is available"',
        ],
        output='screen',
    )

    wait_for_nav_action_cmd = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            'timeout=120; elapsed=0; '
            'until ros2 action list | grep -q "navigate_to_pose"; do '
            '  echo "Waiting for navigate_to_pose... ($elapsed/$timeout x0.1s)"; '
            '  sleep 0.1; elapsed=$((elapsed+1)); '
            '  if [ $elapsed -ge $((timeout * 10)) ]; then '
            '    echo "ERROR: navigate_to_pose not available"; exit 1; '
            '  fi; '
            'done; '
            'echo "navigate_to_pose action server is available"',
        ],
        output='screen',
    )

    explore_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    custom_explore_node = Node(
        package='custom_explore',
        executable='custom_explore_node',
        name='custom_explore_node',
        output='screen',
        parameters=[custom_explore_params, {'use_sim_time': not lidar_connected}],
        remappings=explore_remappings,
    )

    module_test_explore_node = Node(
        package='central_controller',
        executable='module_test_explore_approach',
        name='module_test_explore_approach',
        output='screen',
        parameters=[
            {
                'maps_directory': PathJoinSubstitution([controller_pkg_dir, 'maps']),
                'map_save_basename': 'module02_explore_complete',
                'approach_distance_m': 0.4,
            }
        ],
    )

    if lidar_connected:
        reset_odometry_cmd = ExecuteProcess(
            cmd=[
                'bash',
                '-c',
                'timeout=30; elapsed=0; '
                'until ros2 service list | grep -q "^/reset_odometry$"; do '
                '  echo "Waiting for /reset_odometry... ($elapsed/$timeout x0.1s)"; '
                '  sleep 0.1; elapsed=$((elapsed+1)); '
                '  if [ $elapsed -ge $((timeout * 10)) ]; then '
                '    echo "WARN: /reset_odometry not available; skipping reset"; exit 0; '
                '  fi; '
                'done; '
                'echo "Calling /reset_odometry ..."; '
                'ros2 service call /reset_odometry std_srvs/srv/Trigger "{}" && '
                'echo "/reset_odometry call done"',
            ],
            output='screen',
        )
    else:
        reset_odometry_cmd = ExecuteProcess(
            cmd=['bash', '-c', 'echo "[INFO] Simulation (no LiDAR): skip /reset_odometry"; exit 0'],
            output='screen',
        )

    wait_for_scan_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_scan_cmd,
            on_exit=[
                static_tf_base_to_laser,
                laser_filter_node,
                reset_odometry_cmd,
            ],
        )
    )

    reset_odometry_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=reset_odometry_cmd,
            on_exit=[slam_toolbox_node, slam_configure_activate_cmd],
        )
    )

    after_slam_lifecycle_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=slam_configure_activate_cmd,
            on_exit=[wait_for_map_cmd],
        )
    )

    wait_for_map_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map_cmd,
            on_exit=[nav2_launch, wait_for_nav_action_cmd],
        )
    )

    wait_for_nav_action_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_nav_action_cmd,
            on_exit=[module_test_explore_node, custom_explore_node],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'lidar_connected',
                default_value=lidar_connected_str,
                description='Whether lidar is connected (auto-detected)',
            ),
            SetEnvironmentVariable(name='OMP_NUM_THREADS', value='8'),
            LogInfo(
                msg=['LiDAR connection status: Connected, will start LiDAR node'],
                condition=IfCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"])),
            ),
            LogInfo(
                msg='LiDAR not connected, skipping LiDAR launch (simulation mode, use_sim_time=true)',
                condition=UnlessCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"])),
            ),
            rplidar_launch,
            wait_for_scan_cmd,
            wait_for_scan_handler,
            reset_odometry_handler,
            after_slam_lifecycle_handler,
            wait_for_map_handler,
            wait_for_nav_action_handler,
        ]
    )

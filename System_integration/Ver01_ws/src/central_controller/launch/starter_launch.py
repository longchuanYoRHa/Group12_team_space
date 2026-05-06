"""
Starter launch for the integrated stack.

This launch file intentionally sequences bring-up using small, polled readiness checks
instead of fixed delays. The goal is to make startup robust across:
- real robot (LiDAR present) vs simulation (LiDAR absent / use_sim_time=true)
- variable Nav2 lifecycle activation timing
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Auto-detect LiDAR device.
    # Note: this is a convenience default; users can override via launch arg `lidar_connected`.
    lidar_connected = False
    common_serial_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']

    for port in common_serial_ports:
        if os.path.exists(port):
            lidar_connected = True
            print(f"[INFO] Detected LiDAR device: {port}")
            break

    if not lidar_connected:
        print("[WARN] No LiDAR detected, will skip LiDAR launch (simulation mode)")

    # use_sim_time:
    # - true when no LiDAR (typically simulation)
    # - false when LiDAR is present (real robot)
    use_sim_time_str = 'true' if not lidar_connected else 'false'
    use_sim_time_subst = TextSubstitution(text=use_sim_time_str)

    # Get package directories
    rplidar_pkg_dir = FindPackageShare('rplidar_ros')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    controller_pkg_dir = FindPackageShare('central_controller')
    custom_explore_params = PathJoinSubstitution(
        [FindPackageShare('custom_explore'), 'config', 'params.yaml']
    )

    # Allow manual override while preserving the auto-detected default.
    lidar_connected_str = 'true' if lidar_connected else 'false'
    lidar_connected_config = LaunchConfiguration('lidar_connected', default=lidar_connected_str)

    # 1. Launch RPLidar (without rviz) - only when lidar is connected
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rplidar_pkg_dir, 'launch', 'rplidar_a2m12_launch.py'])
        ),
        condition=IfCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"]))
    )

    # Wait for /scan (tight poll, no fixed delay after).
    # If your "simulation mode" does not publish /scan, this step will time out intentionally.
    wait_for_scan_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             'timeout=120; elapsed=0; '
             'until ros2 topic list | grep -q "^/scan$"; do '
             '  echo "Waiting for /scan topic... ($elapsed/$timeout x0.1s)"; '
             '  sleep 0.1; elapsed=$((elapsed+1)); '
             '  if [ $elapsed -ge $((timeout * 10)) ]; then '
             '    echo "ERROR: /scan topic not available"; exit 1; '
             '  fi; '
             'done; '
             'echo "/scan topic is available"'],
        output='screen'
    )

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

    # Configure+activate slam_toolbox lifecycle once it is discoverable as "unconfigured".
    # We do this via CLI to avoid hard-coding a sleep and to keep the launch logic simple.
    slam_configure_activate_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             'timeout=120; elapsed=0; '
             'until ros2 lifecycle get /slam_toolbox 2>/dev/null | grep -q "unconfigured"; do '
             '  sleep 0.1; elapsed=$((elapsed+1)); '
             '  if [ $elapsed -ge $((timeout * 10)) ]; then '
             '    echo "ERROR: slam_toolbox not unconfigured in time"; exit 1; '
             '  fi; '
             'done; '
             'ros2 lifecycle set /slam_toolbox configure && '
             'ros2 lifecycle set /slam_toolbox activate && '
             'echo "SLAM configure+activate done"'],
        output='screen'
    )

    nav2_params_file = PathJoinSubstitution([controller_pkg_dir, 'config', 'nav2_params_smac2d_rpp.yaml'])

    # NOTE: nav2_bringup's navigation_launch.py does NOT accept a
    # `bt_xml_filename` launch argument. The custom NavigateToPose behavior
    # tree is configured via the `default_nav_to_pose_bt_xml` parameter
    # inside `nav2_params_smac2d_rpp.yaml` (under bt_navigator.ros__parameters),
    # using `$(find-pkg-share central_controller)/config/my_recovery.xml`.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time_subst,
            'params_file': nav2_params_file,
        }.items()
    )

    wait_for_map_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             'timeout=120; elapsed=0; '
             'until ros2 topic list | grep -q "^/map$"; do '
             '  echo "Waiting for /map topic... ($elapsed/$timeout x0.1s)"; '
             '  sleep 0.1; elapsed=$((elapsed+1)); '
             '  if [ $elapsed -ge $((timeout * 10)) ]; then '
             '    echo "ERROR: /map topic not available"; exit 1; '
             '  fi; '
             'done; '
             'echo "/map topic is available"'],
        output='screen'
    )

    # Wait until Nav2 is FULLY up before starting task_manager. We can't rely
    # on a single check because:
    #   - navigate_to_pose action appears as soon as bt_navigator activates,
    #     well before docking_server / collision_monitor finish bonding.
    #   - `ros2 service call /lifecycle_manager_navigation/is_active` blocks
    #     forever while the service does not yet exist, so it MUST be wrapped
    #     in `timeout` and the service presence MUST be checked first.
    #
    # Strategy (three phases, polled with a 180 s wall-clock budget shared via
    # `elapsed` counted in 0.5 s ticks):
    #   1) navigate_to_pose action is advertised
    #   2) /docking_server is in lifecycle state `active` -- this is the last
    #      managed node that lifecycle_manager_navigation activates (matches
    #      "Server docking_server connected with bond" / "Managed nodes are
    #      active" in your logs).
    #   3) /lifecycle_manager_navigation/is_active returns success=True
    #      (final authoritative check that the manager set system_active=true).
    wait_for_nav_action_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             'set -u; '
             'TICK=0.5; MAX_TICKS=360; elapsed=0; '
             # Phase 1: navigate_to_pose action available
             'echo "[wait-nav2] Phase 1/3: waiting for navigate_to_pose action..."; '
             'until ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; do '
             '  sleep $TICK; elapsed=$((elapsed+1)); '
             '  if [ $((elapsed % 10)) -eq 0 ]; then '
             '    echo "[wait-nav2] Phase 1: still waiting navigate_to_pose ($elapsed/$MAX_TICKS x${TICK}s)"; '
             '  fi; '
             '  if [ $elapsed -ge $MAX_TICKS ]; then '
             '    echo "[wait-nav2] ERROR: navigate_to_pose not available in time"; exit 1; '
             '  fi; '
             'done; '
             'echo "[wait-nav2] Phase 1 done: navigate_to_pose action server is up."; '
             # Phase 2: docking_server lifecycle state == active
             'echo "[wait-nav2] Phase 2/3: waiting for /docking_server lifecycle == active..."; '
             'until ros2 lifecycle get /docking_server 2>/dev/null | grep -q "^active"; do '
             '  sleep $TICK; elapsed=$((elapsed+1)); '
             '  if [ $((elapsed % 10)) -eq 0 ]; then '
             '    echo "[wait-nav2] Phase 2: still waiting docking_server active ($elapsed/$MAX_TICKS x${TICK}s)"; '
             '  fi; '
             '  if [ $elapsed -ge $MAX_TICKS ]; then '
             '    echo "[wait-nav2] ERROR: /docking_server not active in time"; exit 1; '
             '  fi; '
             'done; '
             'echo "[wait-nav2] Phase 2 done: /docking_server is active."; '
             # Phase 3: /lifecycle_manager_navigation/is_active service exists, then returns success=True
             'echo "[wait-nav2] Phase 3/3: waiting for /lifecycle_manager_navigation/is_active service..."; '
             'until ros2 service list 2>/dev/null | grep -q "/lifecycle_manager_navigation/is_active"; do '
             '  sleep $TICK; elapsed=$((elapsed+1)); '
             '  if [ $elapsed -ge $MAX_TICKS ]; then '
             '    echo "[wait-nav2] ERROR: is_active service not found in time"; exit 1; '
             '  fi; '
             'done; '
             'echo "[wait-nav2] Phase 3: is_active service found, polling for success=True..."; '
             'until timeout 3 ros2 service call /lifecycle_manager_navigation/is_active '
             '         std_srvs/srv/Trigger "{}" 2>/dev/null '
             '         | grep -Eiq "success[ =:]+true"; do '
             '  sleep $TICK; elapsed=$((elapsed+1)); '
             '  if [ $((elapsed % 10)) -eq 0 ]; then '
             '    echo "[wait-nav2] Phase 3: still waiting is_active=true ($elapsed/$MAX_TICKS x${TICK}s)"; '
             '  fi; '
             '  if [ $elapsed -ge $MAX_TICKS ]; then '
             '    echo "[wait-nav2] ERROR: lifecycle_manager_navigation never reported active"; exit 1; '
             '  fi; '
             'done; '
             'echo "[wait-nav2] Phase 3 done: Nav2 fully active (managed nodes are active)."; '
             # Phase 4: send a short forward nav goal and wait until Nav2 accepts it
             'echo "[wait-nav2] Phase 4/4: sending 0.1m forward nav goal (frame=base_link) until accepted..."; '
             'GOAL="{pose: {header: {frame_id: base_link}, pose: {position: {x: 0.1, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"; '
             'attempt=0; '
             'until timeout 6 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$GOAL" 2>/dev/null | grep -Eiq "goal accepted|accepted"; do '
             '  attempt=$((attempt+1)); '
             '  echo "[wait-nav2] Phase 4: nav goal not accepted yet, retry #$attempt ..."; '
             '  sleep $TICK; '
             'done; '
             'echo "[wait-nav2] Phase 4 done: precheck nav goal accepted."'],
        output='screen'
    )

    task_manager_node = Node(
        package='central_controller',
        executable='task_manager_v4',
        name='task_manager',
        output='screen',
        parameters=[{
            # 'pregrasp_distance': 0.5,  # meters
            # 'preplace_distance': 0.6,  # meters
            # 'stow_pose_x': 0.3,
            # 'stow_pose_y': 0.0,
            # 'stow_pose_z': 0.2,
        }]
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

    if lidar_connected:
        # Real robot: reset odometry once the service is available.
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
        # Simulation convenience: keep the sequencing consistent but no-op the reset.
        reset_odometry_cmd = ExecuteProcess(
            cmd=['bash', '-c', 'echo "[INFO] Simulation (no LiDAR): skip /reset_odometry"; exit 0'],
            output='screen',
        )

    # After /scan: reset odometry first (real robot only) or no-op (sim).
    # SLAM is started AFTER odom reset so that map->odom->base_link stays consistent.
    wait_for_scan_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_scan_cmd,
            on_exit=[reset_odometry_cmd]
        )
    )

    # After odom reset (or sim skip): static TF + laser filter + SLAM + lifecycle script (all parallel)
    reset_odometry_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=reset_odometry_cmd,
            on_exit=[
                static_tf_base_to_laser,
                laser_filter_node,
                slam_toolbox_node,
                slam_configure_activate_cmd,
            ]
        )
    )

    # After SLAM configure+activate script exits, wait for /map
    after_slam_lifecycle_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=slam_configure_activate_cmd,
            on_exit=[wait_for_map_cmd]
        )
    )

    # After /map: Nav2 and navigate_to_pose wait in parallel (no second odom reset)
    wait_for_map_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map_cmd,
            on_exit=[nav2_launch, wait_for_nav_action_cmd],
        )
    )

    wait_for_nav_action_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_nav_action_cmd,
            on_exit=[task_manager_node, custom_explore_node]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_connected',
            default_value=lidar_connected_str,
            description='Whether lidar is connected (auto-detected)'
        ),
        SetEnvironmentVariable(
            name="OMP_NUM_THREADS",
            value="8"
        ),

        LogInfo(
            msg=['LiDAR connection status: Connected, will start LiDAR node'],
            condition=IfCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"]))
        ),
        LogInfo(
            msg='LiDAR not connected, skipping LiDAR launch (simulation mode, use_sim_time=true)',
            condition=UnlessCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"]))
        ),

        rplidar_launch,
        wait_for_scan_cmd,
        wait_for_scan_handler,
        reset_odometry_handler,
        after_slam_lifecycle_handler,
        wait_for_map_handler,
        wait_for_nav_action_handler,
    ])

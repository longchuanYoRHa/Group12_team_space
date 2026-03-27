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
    controller_pkg_dir = FindPackageShare('central_controller')
    custom_explore_params = PathJoinSubstitution(
        [FindPackageShare('custom_explore'), 'config', 'params.yaml']
    )

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_vision = LaunchConfiguration('use_vision', default='false')
    camera_frame_id = LaunchConfiguration('camera_frame_id', default='camera_link')
    
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

    # static_tf_base_to_camera = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_base_to_camera',
    #     arguments=[
    #         '0.14813', '-0.027', '0.05957',  # x, y, z in meters
    #         '0.0', '0.0', '0.0',  # roll, pitch, yaw in radians
    #         'base_link', 'depth_camera' #the name of the camera link need to be clarify 
    #     ],
    #     output='screen',
    #     condition=IfCondition(PythonExpression(["'", lidar_connected_config, "' == 'true'"]))
    # )

    # Laser filter node
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[PathJoinSubstitution([controller_pkg_dir, 'config', 'scan_filter.yaml'])],
    )

    # 4. Launch SLAM toolbox node
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


    # 7. Launch exploration + interest-point approach test node
    module_test_explore_node = Node(
        package='central_controller',
        executable='module_test_explore_approach',
        name='module_test_explore_approach',
        output='screen',
        parameters=[{
            'maps_directory': PathJoinSubstitution([controller_pkg_dir, 'maps']),
            'map_save_basename': 'module02_explore_complete',
            'approach_distance_m': 0.4,
            'use_sim_time': use_sim_time,
        }]
    )
    
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
                static_tf_after_scan,
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
        period=3.0,  # wait for SLAM node after 5 seconds (2 seconds start + 3 seconds wait)
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

    # Reset odometry before starting Nav2 (Leo Rover firmware service: /reset_odometry)
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

    # 8. After /map topic available, reset odom once, then start Nav2
    nav2_after_map = TimerAction(
        period=0.1,  # short delay
        actions=[nav2_launch],
        condition=UnlessCondition(PythonExpression(["'", use_sim_time, "' == 'true'"]))
    )

    # In simulation, skip reset_odometry and start Nav2 directly after /map is ready.
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
            on_exit=[nav2_after_map],
        )
    )

    # Frontier exploration: custom_explore（默认等待 explore/resume=true 后才开始，与 task_manager_v2 一致）
    explore_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    custom_explore_node = Node(
        package='custom_explore',
        executable='custom_explore_node',
        name='custom_explore_node',
        output='screen',
        parameters=[custom_explore_params, {'use_sim_time': use_sim_time}],
        remappings=explore_remappings,
    )

    # 11. After navigate_to_pose available, delay start explore test node
    module_test_after_nav_action = TimerAction(
        period=3.0,  # give Nav2 time to be ready
        actions=[module_test_explore_node, custom_explore_node]
    )

    # 9. After Nav2 starts, wait for navigate_to_pose action available, then start test node
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
    
    wait_for_nav_action_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_nav_action_cmd,
            on_exit=[module_test_after_nav_action]
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
        DeclareLaunchArgument(
            'use_vision',
            default_value='true',
            description='Set true to start rover_vision_node (RealSense + YOLO for pick/place targets)'
        ),

        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='camera_link',
            description='TF frame id for vision 3D points (e.g. camera_link or D435i_camera_link)'
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
        wait_for_map_handler,        # after /map available, reset odom before Nav2
        reset_odometry_handler,      # after reset odom, start Nav2
        nav2_handler,                # after /map available, delay wait for navigate_to_pose action
        wait_for_nav_action_handler, # after navigate_to_pose action available, start explore
        # explore_handler,              # after action available, delay start task manager and rviz
    ])

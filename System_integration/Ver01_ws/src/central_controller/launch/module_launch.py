import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo
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
        parameters=[slam_toolbox_params_file, {'use_sim_time': use_sim_time}],
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

    # Poll until slam_toolbox lifecycle becomes unconfigured, then configure + activate immediately.
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


    # 7. Launch Task Manager State Machine Node
    task_manager_node = Node(
        package='central_controller',
        # executable='task_manager',
        executable='module_test_docking',
        name='module_test_docking',
        output='screen',
        parameters=[{
            'camera_frame_id': camera_frame_id,
            'dock_type': 'simple_non_charging_dock',
            'use_external_detection_pose': True,
            # 对齐 Nav2 Docking Server 默认外部检测话题名；
            # 节点内部会同时兼容发布到 /docking_server/detected_dock_pose。
            'external_detection_pose_topic': 'detected_dock_pose',
            'external_detection_pose_frame': 'odom',
            'use_sim_time': use_sim_time,
        }]
    )
    
    # 1. Wait for /scan topic available, then start static transform, filter and SLAM.
    wait_for_scan_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_scan_cmd,
            on_exit=[
                static_tf_base_to_laser,
                laser_filter_node,
                slam_toolbox_node,
                slam_configure_activate_cmd,
            ]
        )
    )

    # 2. After SLAM configure+activate completes, wait for /map.
    activate_slam_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=slam_configure_activate_cmd,
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

    reset_odometry_skip_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'echo "[INFO] Simulation mode: skip /reset_odometry"; exit 0'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_sim_time, "' == 'true'"]))
    )

    wait_for_map_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map_cmd,
            on_exit=[reset_odometry_cmd, reset_odometry_skip_cmd]
        )
    )

    reset_odometry_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=reset_odometry_cmd,
            on_exit=[nav2_launch, wait_for_nav_action_cmd],
        )
    )

    reset_odometry_skip_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=reset_odometry_skip_cmd,
            on_exit=[nav2_launch, wait_for_nav_action_cmd],
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

    # 3. After navigate_to_pose action becomes available, start task manager and explore.
    wait_for_nav_action_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_nav_action_cmd,
            on_exit=[task_manager_node, custom_explore_node]
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
        
        # 3-7. start subsequent nodes through event handler chain
        wait_for_scan_handler,       # after /scan available, start TF/filter/SLAM and lifecycle transitions
        activate_slam_handler,       # after SLAM activate, wait for /map
        wait_for_map_handler,        # after /map available, reset odom or skip in sim
        reset_odometry_handler,      # after reset odom, start Nav2 and wait for action
        reset_odometry_skip_handler, # simulation path: start Nav2 and wait for action
        wait_for_nav_action_handler, # after navigate_to_pose action available, start task manager and explore
    ])

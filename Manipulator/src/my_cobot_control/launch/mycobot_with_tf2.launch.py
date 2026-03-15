#!/usr/bin/env python3
"""
Launch file for MyCobot with TF2 coordinate transforms.

This launch file:
1. Broadcasts static TF2 transforms for arm, and gripper
2. Starts the TF2-enabled mycobot controller

TF2 Frame Hierarchy (with use_camera_link=true):
base_link (rover base)
  └── camera_link
      └── g_base (arm base)
          └── joint1 -> ... -> joint6_flange -> gripper_tip

TF2 Frame Hierarchy (with use_camera_link=false, default):
base_link (rover base)
  └── g_base (arm base)
        └── joint1 -> ... -> joint6_flange -> gripper_tip
                    
Usage:
    # Real hardware (default)
    ros2 launch my_cobot_control mycobot_with_tf2.launch.py
    
    # Development with GUI
    ros2 launch my_cobot_control mycobot_with_tf2.launch.py use_mock:=true
    
    # With camera_link TF chain
    ros2 launch my_cobot_control mycobot_with_tf2.launch.py use_camera_link:=true

    # Override base_link -> camera_link and camera_link -> g_base offsets
    ros2 launch my_cobot_control mycobot_with_tf2.launch.py \
        use_camera_link:=true \
        base_to_camera_x:=0.10 base_to_camera_z:=0.20 \
        camera_to_arm_x:=0.0379 camera_to_arm_y:=0.0641 camera_to_arm_z:=-0.0486
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ========================================================================
    # Read URDF
    # ========================================================================
    pkg_share = get_package_share_directory('my_cobot_control')
    urdf_file = os.path.join(
        pkg_share, 'urdf', 'mycobot_280_pi', 
        'mycobot_280_pi_adaptive_gripper.urdf'
    )
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ========================================================================
    # Launch Arguments
    # ========================================================================
    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='false',  # Default to false for real hardware
        description='Use mock/GUI for joint states (true) or real hardware (false)'
    )

    use_camera_link_arg = DeclareLaunchArgument(
        'use_camera_link',
        default_value='false',  # Default: base_link -> g_base directly
        description='Use camera_link in TF tree (true: base_link->camera_link->g_base, false: base_link->g_base)'
    )

    base_x_arg = DeclareLaunchArgument(
        'base_x', default_value='0.06494',
        description='Base-to-arm X offset for base_link->g_base (m)')
    base_y_arg = DeclareLaunchArgument(
        'base_y', default_value='0.08399',
        description='Base-to-arm Y offset for base_link->g_base (m)')
    base_z_arg = DeclareLaunchArgument(
        'base_z', default_value='0.01097',
        description='Base-to-arm Z offset for base_link->g_base (m)')
    base_roll_arg = DeclareLaunchArgument(
        'base_roll', default_value='0.0',
        description='Base-to-arm roll for base_link->g_base (rad)')
    base_pitch_arg = DeclareLaunchArgument(
        'base_pitch', default_value='0.0',
        description='Base-to-arm pitch for base_link->g_base (rad)')
    base_yaw_arg = DeclareLaunchArgument(
        'base_yaw', default_value='0.0',
        description='Base-to-arm yaw for base_link->g_base (rad)')

    base_to_camera_x_arg = DeclareLaunchArgument(
        'base_to_camera_x', default_value='0.027',
        description='Base-to-camera X offset for base_link->camera_link (m)')
    base_to_camera_y_arg = DeclareLaunchArgument(
        'base_to_camera_y', default_value='0.14813',
        description='Base-to-camera Y offset for base_link->camera_link (m)')
    base_to_camera_z_arg = DeclareLaunchArgument(
        'base_to_camera_z', default_value='0.05957',
        description='Base-to-camera Z offset for base_link->camera_link (m)')
    base_to_camera_roll_arg = DeclareLaunchArgument(
        'base_to_camera_roll', default_value='0.5236', # 30 degrees in radians
        description='Base-to-camera roll for base_link->camera_link (rad)')
    base_to_camera_pitch_arg = DeclareLaunchArgument(
        'base_to_camera_pitch', default_value='0.0',
        description='Base-to-camera pitch for base_link->camera_link (rad)')
    base_to_camera_yaw_arg = DeclareLaunchArgument(
        'base_to_camera_yaw', default_value='0.0',
        description='Base-to-camera yaw for base_link->camera_link (rad)')

    camera_to_arm_x_arg = DeclareLaunchArgument(
        'camera_to_arm_x', default_value='0.0379',
        description='Camera-to-arm X offset for camera_link->g_base (m)')
    camera_to_arm_y_arg = DeclareLaunchArgument(
        'camera_to_arm_y', default_value='0.0641',
        description='Camera-to-arm Y offset for camera_link->g_base (m)')
    camera_to_arm_z_arg = DeclareLaunchArgument(
        'camera_to_arm_z', default_value='-0.0486',
        description='Camera-to-arm Z offset for camera_link->g_base (m)')
    camera_to_arm_roll_arg = DeclareLaunchArgument(
        'camera_to_arm_roll', default_value='0.0',
        description='Camera-to-arm roll for camera_link->g_base (rad)')
    camera_to_arm_pitch_arg = DeclareLaunchArgument(
        'camera_to_arm_pitch', default_value='-0.5236',
        description='Camera-to-arm pitch for camera_link->g_base (rad)')
    camera_to_arm_yaw_arg = DeclareLaunchArgument(
        'camera_to_arm_yaw', default_value='0.0',
        description='Camera-to-arm yaw for camera_link->g_base (rad)')

    # ========================================================================
    # Robot State Publisher (Publish joint state in TF)
    # ========================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # ========================================================================
    # Joint State Publisher GUI (Development environment, manual joint control)
    # ========================================================================
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mock'))
    )

    # Fallback: non-GUI joint state publisher for when GUI is not available
    # but also needed to provide a /joint_states base for robot_state_publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_mock'))
    )

    # ========================================================================
    # Static TF: base_link -> camera_link (when use_camera_link=true)
    # ========================================================================
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            LaunchConfiguration('base_to_camera_x'),
            LaunchConfiguration('base_to_camera_y'),
            LaunchConfiguration('base_to_camera_z'),
            LaunchConfiguration('base_to_camera_roll'),
            LaunchConfiguration('base_to_camera_pitch'),
            LaunchConfiguration('base_to_camera_yaw'),
            'base_link',
            'camera_link'
        ],
        condition=IfCondition(LaunchConfiguration('use_camera_link'))
    )

    # ========================================================================
    # Static TF: camera_link -> g_base (when use_camera_link=true)
    # ========================================================================
    camera_to_arm_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_arm_tf',
        arguments=[
            LaunchConfiguration('camera_to_arm_x'),
            LaunchConfiguration('camera_to_arm_y'),
            LaunchConfiguration('camera_to_arm_z'),
            LaunchConfiguration('camera_to_arm_roll'),
            LaunchConfiguration('camera_to_arm_pitch'),
            LaunchConfiguration('camera_to_arm_yaw'),
            'camera_link',
            'g_base'  # arm_base_link in URDF is g_base
        ],
        condition=IfCondition(LaunchConfiguration('use_camera_link'))
    )

    # ========================================================================
    # Static TF: base_link -> g_base (when use_camera_link=false, default)
    # ========================================================================
    base_to_arm_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_arm_tf',
        arguments=[
            LaunchConfiguration('base_x'),
            LaunchConfiguration('base_y'),
            LaunchConfiguration('base_z'),
            LaunchConfiguration('base_roll'),
            LaunchConfiguration('base_pitch'),
            LaunchConfiguration('base_yaw'),
            'base_link',
            'g_base'
        ],
        condition=UnlessCondition(LaunchConfiguration('use_camera_link'))
    )

    # ========================================================================
    # Static TF: joint6_flange -> gripper_tip
    # ========================================================================
    gripper_tip_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gripper_tip_tf',
        arguments=[
            '0.0', '0.0', '0.079',  # 34mm + 45mm
            '0', '0', '0',
            'joint6_flange',
            'gripper_tip'
        ]
    )

    # ========================================================================
    # MyCobot Controller
    # ========================================================================
    mycobot_controller = Node(
        package='my_cobot_control',
        executable='mycobot_controller_tf2',
        name='mycobot_controller',
        namespace='arm',
        output='screen',
        parameters=[{
            'camera_frame': 'camera_link',
            'arm_base_frame': 'g_base',  # Name in URDF
            'flange_frame': 'joint6_flange',
            'gripper_tip_frame': 'gripper_tip',
            'tf_timeout': 1.0,
            'compensate_gripper_offset': True,
            'safe_z': 200.0,
            'move_speed': 30,
            'gripper_speed': 80,
            'end_rx': -178.0,
            'end_ry': 0.0,
            'end_rz': -60.0,
            'home_angles': [0.0, 0.0, 0.0, 0.0, 0.0, -45], # degree
        }]
    )

    return LaunchDescription([
        use_mock_arg,
        use_camera_link_arg,
        base_x_arg,
        base_y_arg,
        base_z_arg,
        base_roll_arg,
        base_pitch_arg,
        base_yaw_arg,
        base_to_camera_x_arg,
        base_to_camera_y_arg,
        base_to_camera_z_arg,
        base_to_camera_roll_arg,
        base_to_camera_pitch_arg,
        base_to_camera_yaw_arg,
        camera_to_arm_x_arg,
        camera_to_arm_y_arg,
        camera_to_arm_z_arg,
        camera_to_arm_roll_arg,
        camera_to_arm_pitch_arg,
        camera_to_arm_yaw_arg,
        
        robot_state_publisher,
        # mock mode: manual control joint states via GUI
        joint_state_publisher_gui,
        # real hardware mode: default zero position, wait for arm node to override
        joint_state_publisher,      
        
        base_to_camera_tf,
        camera_to_arm_tf,
        base_to_arm_tf,
        gripper_tip_tf,
        
        mycobot_controller,
    ])

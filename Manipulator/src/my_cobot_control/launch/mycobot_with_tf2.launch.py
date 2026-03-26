#!/usr/bin/env python3
"""
Launch file for MyCobot with TF2 coordinate transforms.

This launch file:
1. Broadcasts static TF2 transforms for arm, and gripper
2. Starts the TF2-enabled mycobot controller

TF2 Frame Hierarchy:
base_link (rover base)
  ├── camera_link
  └── g_base (arm base)
        └── joint1 -> ... -> joint6_flange -> gripper_tip
                    
Usage:
    # Real hardware (default)
    ros2 launch my_cobot_control mycobot_with_tf2.launch.py
    
    # Development with GUI
    ros2 launch my_cobot_control mycobot_with_tf2.launch.py use_mock:=true

    # Override base_link -> camera_link and base_link -> g_base offsets
    ros2 launch my_cobot_control mycobot_with_tf2.launch.py \
        base_to_camera_x:=0.10 base_to_camera_z:=0.20 \
        base_x:=0.08399 base_y:=0.06494 base_z:=0.01097
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
        description='Mock mode: true=simulation/dev (no hardware), false=real hardware'
    )

    base_x_arg = DeclareLaunchArgument(
        'base_x', default_value='0.05500',
        description='Base-to-arm X offset for base_link->g_base (m)')
    base_y_arg = DeclareLaunchArgument(
        'base_y', default_value='0.03594',
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
        'base_to_camera_x', default_value='0.158',
        description='Base-to-camera X offset for base_link->camera_link (m)')
    base_to_camera_y_arg = DeclareLaunchArgument(
        'base_to_camera_y', default_value='0.007',
        description='Base-to-camera Y offset for base_link->camera_link (m)')
    base_to_camera_z_arg = DeclareLaunchArgument(
        'base_to_camera_z', default_value='0.081',
        description='Base-to-camera Z offset for base_link->camera_link (m)')
    base_to_camera_roll_arg = DeclareLaunchArgument(
        'base_to_camera_roll', default_value='-1.9199', # 20 + 90 degrees in radians
        description='Base-to-camera roll for base_link->camera_link (rad)')
    base_to_camera_pitch_arg = DeclareLaunchArgument(
        'base_to_camera_pitch', default_value='0.0',
        description='Base-to-camera pitch for base_link->camera_link (rad)')
    base_to_camera_yaw_arg = DeclareLaunchArgument(
        'base_to_camera_yaw', default_value='-1.5708', # 90 degrees in radians
        description='Base-to-camera yaw for base_link->camera_link (rad)')

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
    # Static TF: base_link -> camera_link (default)
    # ========================================================================
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '--x', LaunchConfiguration('base_to_camera_x'),
            '--y', LaunchConfiguration('base_to_camera_y'),
            '--z', LaunchConfiguration('base_to_camera_z'),
            '--yaw', LaunchConfiguration('base_to_camera_yaw'),
            '--pitch', LaunchConfiguration('base_to_camera_pitch'),
            '--roll', LaunchConfiguration('base_to_camera_roll'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ],
    )

    # ========================================================================
    # Static TF: base_link -> g_base (default)
    # ========================================================================
    base_to_arm_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_arm_tf',
        arguments=[
            '--x', LaunchConfiguration('base_x'),
            '--y', LaunchConfiguration('base_y'),
            '--z', LaunchConfiguration('base_z'),
            '--yaw', LaunchConfiguration('base_yaw'),
            '--pitch', LaunchConfiguration('base_pitch'),
            '--roll', LaunchConfiguration('base_roll'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'g_base'
        ]
    )

    # ========================================================================
    # Static TF: joint6_flange -> gripper_tip
    # ========================================================================
    gripper_tip_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gripper_tip_tf',
        arguments=[
            '--x', '0.0', 
            '--y', '0.010',  # 10mm offset from flange center
            '--z', '0.095',  # 95mm offset from flange center
            '--yaw', '0.0', 
            '--pitch', '0.0', 
            '--roll', '0.0',
            '--frame-id', 'joint6_flange',
            '--child-frame-id', 'gripper_tip'
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
        emulate_tty=True,
        parameters=[{
            'camera_frame': 'camera_link',
            'arm_base_frame': 'g_base',  # Name in URDF
            'flange_frame': 'joint6_flange',
            'gripper_tip_frame': 'gripper_tip',
            'tf_timeout': 1.0,
            'compensate_gripper_offset': True,
            'safe_z': 0.200,  # Minimum Z height to avoid collisions (m)
            'move_speed': 30,
            'gripper_speed': 80,
            'end_rx': -180.0, # End-effector orientation to keep gripper vertical to ground (degrees)
            'end_ry': 0.0,
            'end_rz': -135.0,
            'home_angles': [0.0, 0.0, 0.0, 0.0, 0.0, 45.0], # degree
        }]
    )

    return LaunchDescription([
        use_mock_arg,

        # Base to arm TF parameters (default)
        base_x_arg,
        base_y_arg,
        base_z_arg,
        base_roll_arg,
        base_pitch_arg,
        base_yaw_arg,
        # Base to camera TF parameters (default)
        base_to_camera_x_arg,
        base_to_camera_y_arg,
        base_to_camera_z_arg,
        base_to_camera_roll_arg,
        base_to_camera_pitch_arg,
        base_to_camera_yaw_arg,
        
        robot_state_publisher,
        # mock mode: manual control joint states via GUI
        joint_state_publisher_gui,
        # real hardware mode: default zero position, wait for arm node to override
        joint_state_publisher,      
        
        base_to_camera_tf,
        base_to_arm_tf,
        gripper_tip_tf,
        
        mycobot_controller,
    ])

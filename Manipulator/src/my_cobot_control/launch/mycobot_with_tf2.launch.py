#!/usr/bin/env python3
"""
Launch file for MyCobot with TF2 coordinate transforms.

This launch file:
1. Broadcasts static TF2 transforms for camera, arm, and gripper
2. Starts the TF2-enabled mycobot controller

TF2 Frame Hierarchy:
camera_link
  └── arm_base_link (g_base in URDF)
        └── joint6_flange
              └── gripper_base
                    └── gripper_tip
                    
Usage:
    ros2 launch my_cobot_control mycobot_with_tf2.launch.py
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
        default_value='true',  # Default to true for development environment
        description='Use mock/GUI for joint states (true) or real hardware (false)'
    )

    camera_x_arg = DeclareLaunchArgument(
        'camera_x', default_value='0.0379',
        description='Camera X offset from arm base (m), positive = front')
    camera_y_arg = DeclareLaunchArgument(
        'camera_y', default_value='0.0641',
        description='Camera Y offset from arm base (m), positive = right')
    camera_z_arg = DeclareLaunchArgument(
        'camera_z', default_value='-0.0486',
        description='Camera Z offset from arm base (m), positive = down')
    camera_pitch_arg = DeclareLaunchArgument(
        'camera_pitch', default_value='-0.5236')  # -30° in rad

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
    # In real environment, joint states are published by mycobot_controller


    # ========================================================================
    # Static TF: camera_link -> g_base （arm_base_link in URDF）
    # ========================================================================
    camera_to_arm_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_arm_tf',
        arguments=[
            LaunchConfiguration('camera_x'),
            LaunchConfiguration('camera_y'),
            LaunchConfiguration('camera_z'),
            '0',  # roll (rad)
            LaunchConfiguration('camera_pitch'),
            '0',
            'camera_link',
            'g_base'  # army_base_link in URDF is g_base
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
            'safe_z': 250.0,
            'move_speed': 40,
            'gripper_speed': 80,
            'end_rx': -178.0,
            'end_ry': 0.0,
            'end_rz': 0.0,
            'home_angles': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }]
    )

    return LaunchDescription([
        use_mock_arg,
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        camera_pitch_arg,
        
        robot_state_publisher,
        joint_state_publisher_gui,  # Only for development/testing
        
        camera_to_arm_tf,
        gripper_tip_tf,
        
        mycobot_controller,
    ])

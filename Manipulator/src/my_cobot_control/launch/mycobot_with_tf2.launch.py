#!/usr/bin/env python3
"""
Launch file for MyCobot with TF2 coordinate transforms.

This launch file:
1. Broadcasts static TF2 transforms for camera, arm, and gripper
2. Starts the TF2-enabled mycobot controller

Usage:
    ros2 launch my_cobot_control mycobot_with_tf2.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import math


def generate_launch_description():
    # Declare launch arguments
    camera_x_arg = DeclareLaunchArgument(
        'camera_x', default_value='0.2',
        description='Camera X offset from rover base (m)')
    camera_y_arg = DeclareLaunchArgument(
        'camera_y', default_value='-0.1',
        description='Camera Y offset from rover base (m)')
    camera_z_arg = DeclareLaunchArgument(
        'camera_z', default_value='0.3',
        description='Camera Z offset from rover base (m)')
    camera_pitch_arg = DeclareLaunchArgument(
        'camera_pitch', default_value='-30.0',
        description='Camera pitch angle (degrees, negative = tilted down)')

    # TF2 static broadcasters
    # Transform 1: rover_base -> camera_link
    camera_tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_broadcaster',
        arguments=[
            LaunchConfiguration('camera_x'),
            LaunchConfiguration('camera_y'),
            LaunchConfiguration('camera_z'),
            '0',  # roll
            LaunchConfiguration('camera_pitch'),
            '0',  # yaw
            'rover_base',
            'camera_link'
        ],
        parameters=[{'use_sim_time': False}]
    )

    # Transform 2: rover_base -> arm_base_link
    # Assuming arm is mounted at center of rover, 50mm above base plate
    arm_base_tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='arm_base_tf_broadcaster',
        arguments=[
            '0.0', '0.0', '0.05',  # x, y, z (m)
            '0', '0', '0',          # roll, pitch, yaw (rad)
            'rover_base',
            'arm_base_link'
        ],
        parameters=[{'use_sim_time': False}]
    )

    # Transform 3: joint6_flange -> gripper_base
    # From URDF: <origin xyz="0 0 0.034" rpy="1.579 0 0"/>
    gripper_base_tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gripper_base_tf_broadcaster',
        arguments=[
            '0.0', '0.0', '0.034',  # +34mm in Z
            '1.579', '0', '0',      # 90 degrees rotation around X
            'joint6_flange',
            'gripper_base'
        ],
        parameters=[{'use_sim_time': False}]
    )

    # Transform 4: gripper_base -> gripper_tip
    # Approximate gripper finger length
    gripper_tip_tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gripper_tip_tf_broadcaster',
        arguments=[
            '0.0', '0.0', '0.045',  # +45mm in Z (finger length)
            '0', '0', '0',
            'gripper_base',
            'gripper_tip'
        ],
        parameters=[{'use_sim_time': False}]
    )

    # MyCobot TF2 controller
    mycobot_controller = Node(
        package='my_cobot_control',
        executable='mycobot_controller_tf2',
        name='mycobot_controller',
        namespace='arm',
        output='screen',
        parameters=[{
            'camera_frame': 'camera_link',
            'arm_base_frame': 'arm_base_link',
            'flange_frame': 'joint6_flange',
            'gripper_tip_frame': 'gripper_tip',
            'tf_timeout': 1.0,
            'compensate_gripper_offset': True,
            'safe_z': 250.0,
            'move_speed': 40,
            'gripper_speed': 80,
        }]
    )

    return LaunchDescription([
        # Launch arguments
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        camera_pitch_arg,

        # TF2 broadcasters
        camera_tf_broadcaster,
        arm_base_tf_broadcaster,
        gripper_base_tf_broadcaster,
        gripper_tip_tf_broadcaster,

        # Controller
        mycobot_controller,
    ])

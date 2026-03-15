#!/usr/bin/env python3

"""
Launch file for the MyCobot 280 Pi arm controller.

Starts:
  1. robot_state_publisher  — publishes URDF to /arm/robot_description
  2. mycobot_controller     — subscribes to /arm/target_pick & /arm/target_place
  3. (optional) rviz2       — visualisation

All nodes run under namespace 'arm' to avoid topic clashes with Leo Rover.
Fixed camera to arm TF frames
Usage:
  ros2 launch my_cobot_control arm_controller.launch.py
  ros2 launch my_cobot_control arm_controller.launch.py rviz:=true
  ros2 launch my_cobot_control arm_controller.launch.py safe_z:=250.0 move_speed:=40
"""

import os
from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ---- Launch arguments ---------------------------------------------------
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(
            get_package_share_path('mycobot_description'),
            'urdf/mycobot_280_pi/mycobot_280_pi_adaptive_gripper.urdf',
        ),
        description='Path to URDF file',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 for visualisation',
    )

    rvizconfig_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=os.path.join(
            get_package_share_path('mycobot_280pi'),
            'config/mycobot_pi.rviz',
        ),
        description='RViz config file',
    )

    safe_z_arg = DeclareLaunchArgument(
        'safe_z', default_value='220.0',
        description='Safe Z height in mm',
    )

    move_speed_arg = DeclareLaunchArgument(
        'move_speed', default_value='50',
        description='Arm movement speed (1-100)',
    )

    gripper_speed_arg = DeclareLaunchArgument(
        'gripper_speed', default_value='80',
        description='Gripper speed (1-100)',
    )

    # ---- Robot description --------------------------------------------------
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='arm',
        parameters=[{'robot_description': robot_description}],
        # Remap so it reads joint_states from our controller in the same ns
        remappings=[('joint_states', 'joint_states')],
    )

    # ---- Arm controller node ------------------------------------------------
    controller_node = Node(
        package='my_cobot_control',
        executable='mycobot_controller',
        name='mycobot_controller',
        namespace='arm',
        output='screen',
        parameters=[{
            'safe_z': LaunchConfiguration('safe_z'),
            'move_speed': LaunchConfiguration('move_speed'),
            'gripper_speed': LaunchConfiguration('gripper_speed'),
        }],
    )

    # ---- Optional RViz ------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace='arm',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        rvizconfig_arg,
        safe_z_arg,
        move_speed_arg,
        gripper_speed_arg,
        robot_state_publisher,
        controller_node,
        rviz_node,
    ])

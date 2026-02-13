#!/usr/bin/env python3

# This is the RViz demo launch node for myCobot 280, 
# simulating pick and place actions.

import os
from ament_index_python import get_package_share_path
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    res = []

    # URDF model
    model_launch_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_path("mycobot_description"),
            "urdf/mycobot_280_pi/mycobot_280_pi_adaptive_gripper.urdf"
        )
    )
    res.append(model_launch_arg)

    # RViz config
    rvizconfig_launch_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=os.path.join(
            get_package_share_path("mycobot_280pi"),
            "config/mycobot_pi.rviz"
        )
    )
    res.append(rvizconfig_launch_arg)

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    res.append(robot_state_publisher_node)

    # RViz
    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=['-d', LaunchConfiguration("rvizconfig")],
    )
    res.append(rviz_node)

    # Your pick and place visualizer
    pick_and_place_node = Node(
        package="my_cobot_control",
        executable="pick_and_place_rviz",
        name="pick_and_place_visualizer",
        output="screen"
    )
    res.append(pick_and_place_node)

    return LaunchDescription(res)
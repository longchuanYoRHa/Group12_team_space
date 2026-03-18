#!/usr/bin/env python3
"""
Launch file for MyCobot 280 Pi + Adaptive Gripper in Gazebo simulation.
Uses the merged Gazebo-compatible URDF: mycobot_280_gazebo_gripper.urdf
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    LogInfo,
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "mycobot280_pi"
    pkg_share = get_package_share_directory(pkg_name)
    # Required for resolving gripper mesh package:// URIs
    mycobot_desc_share = get_package_share_directory("mycobot_description")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # -------------------------------------------------------------------------
    # File paths — all resolved via pkg_share, no hardcoded absolute paths
    # -------------------------------------------------------------------------
    # Merged Gazebo-compatible URDF with adaptive gripper
    urdf_path = os.path.join(pkg_share, "urdf", "mycobot_280_gazebo_gripper.urdf")

    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(
            f"Merged gripper URDF not found: {urdf_path}\n"
            f"Please ensure mycobot_280_gazebo_gripper.urdf is placed under "
            f"src/mycobot280_pi/urdf/ and the package has been rebuilt."
        )

    controllers_yaml = os.path.join(
        pkg_share, "config", "mycobot_280_controllers.yaml"
    )
    world_path = os.path.join(pkg_share, "worlds", "default.world")
    world_to_use = world_path if os.path.exists(world_path) else "empty.sdf"

    # -------------------------------------------------------------------------
    # Read URDF and resolve all package:// URIs to absolute paths
    # (Gazebo does not resolve package:// URIs natively)
    # -------------------------------------------------------------------------
    with open(urdf_path, "r") as fh:
        robot_description_str = fh.read()

    # Replace controller config path placeholder
    robot_description_str = robot_description_str.replace(
        "MYCOBOT_CONFIG_PATH", controllers_yaml
    )
    # Resolve arm mesh URIs
    robot_description_str = robot_description_str.replace(
        "package://mycobot280_pi/", pkg_share + "/"
    )
    # Resolve gripper mesh URIs (from mycobot_description / mycobot_ros2)
    robot_description_str = robot_description_str.replace(
        "package://mycobot_description/", mycobot_desc_share + "/"
    )

    robot_description_param = {"robot_description": robot_description_str}

    # -------------------------------------------------------------------------
    # Robot State Publisher
    # -------------------------------------------------------------------------
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param, {"use_sim_time": use_sim_time}],
    )

    # -------------------------------------------------------------------------
    # Gazebo simulator
    # -------------------------------------------------------------------------
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_launch_file = os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_file),
        launch_arguments={
            "gz_args": TextSubstitution(text=f"-r -v 4 {world_to_use}")
        }.items(),
    )

    # -------------------------------------------------------------------------
    # Spawn robot entity in Gazebo
    # -------------------------------------------------------------------------
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="ros_gz_create_mycobot_gripper",
        output="screen",
        arguments=[
            "-world", "default",
            "-name",  "mycobot_280_gripper",
            "-param", "robot_description",
        ],
        parameters=[robot_description_param],
    )

    # -------------------------------------------------------------------------
    # ROS-Gazebo clock bridge
    # -------------------------------------------------------------------------
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # -------------------------------------------------------------------------
    # Controller spawners
    # Startup order: joint_state_broadcaster -> arm controller -> gripper controller
    # -------------------------------------------------------------------------
    spawner_js = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_trajectory_controller",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    spawner_gripper = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_gripper_controller",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawn complete -> 5s delay -> start joint_state_broadcaster
    start_js_after_spawn = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gz_spawn_entity,
            on_start=[TimerAction(period=5.0, actions=[spawner_js])],
        )
    )
    # joint_state_broadcaster exits -> 2s delay -> start arm controller
    start_arm_after_js_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_js,
            on_exit=[TimerAction(period=2.0, actions=[spawner_arm])],
        )
    )
    # Arm controller exits -> 2s delay -> start gripper controller
    start_gripper_after_arm_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_arm,
            on_exit=[TimerAction(period=2.0, actions=[spawner_gripper])],
        )
    )

    info = LogInfo(msg=[f"Launching MyCobot 280 Pi + Adaptive Gripper. URDF: {urdf_path}"])

    return LaunchDescription([
        declare_use_sim_time,
        bridge,
        info,
        rsp_node,
        gz_sim_launch,
        gz_spawn_entity,
        start_js_after_spawn,
        start_arm_after_js_exit,
        start_gripper_after_arm_exit,
    ])
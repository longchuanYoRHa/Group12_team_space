#!/usr/bin/env python3


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
    # Adjust if your package name is different
    pkg_name = "mycobot280_pi"
    pkg_share = get_package_share_directory(pkg_name)

    # -------------------------------------------------------------------------
    # Launch arguments
    # -------------------------------------------------------------------------
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # -------------------------------------------------------------------------
    # Files (computed at launch parse time)
    # -------------------------------------------------------------------------
    urdf_path = os.path.join(pkg_share, "urdf", "mycobot_280_gazebo.urdf")

    # Optional world: use package world if available, otherwise empty.sdf
    world_path = os.path.join(pkg_share, "worlds", "default.world")
    world_to_use = world_path if os.path.exists(world_path) else "empty.sdf"

    # Sanity checks
    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    # -------------------------------------------------------------------------
    # Read URDF into robot_description
    # -------------------------------------------------------------------------
    with open(urdf_path, "r") as fh:
        robot_description_str = fh.read()
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
    # Launch Gazebo (ros_gz_sim)
    # -------------------------------------------------------------------------
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_launch_file = os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
    if not os.path.exists(gz_launch_file):
        raise FileNotFoundError(
            f"Could not find ros_gz_sim launch file: {gz_launch_file}"
        )

    gz_args_text = f"-r {world_to_use}"
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_file),
        launch_arguments={"gz_args": TextSubstitution(text=gz_args_text)}.items(),
    )

    # -------------------------------------------------------------------------
    # Spawn entity in Gazebo using /robot_description
    # -------------------------------------------------------------------------
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="ros_gz_create_mycobot",
        output="screen",
        arguments=[
            "-world",
            "default",
            "-name",
            "mycobot_280",
            "-param",
            "robot_description",
        ],
        parameters=[robot_description_param],
    )

    # -------------------------------------------------------------------------
    # /clock bridge (fixed closing bracket in type description)
    # -------------------------------------------------------------------------
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # -------------------------------------------------------------------------
    # Controller spawners with delays
    # -------------------------------------------------------------------------
    spawner_js = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_trajectory_controller",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Wait 30s after the spawn process starts, then start joint_state_broadcaster
    start_js_after_spawn = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gz_spawn_entity,
            on_start=[TimerAction(period=2.0, actions=[spawner_js])],
        )
    )

    # After JS broadcaster exits, wait 2s and start arm controller
    start_arm_after_js_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_js,
            on_exit=[TimerAction(period=2.0, actions=[spawner_arm])],
        )
    )

    # -------------------------------------------------------------------------
    # Log info
    # -------------------------------------------------------------------------
    info = LogInfo(
        msg=[
            "Launching Gazebo, publishing URDF, spawning MyCobot 280, and starting controllers."
        ]
    )

    # -------------------------------------------------------------------------
    # LaunchDescription
    # -------------------------------------------------------------------------
    ld = LaunchDescription(
        [
            declare_use_sim_time,
            bridge,
            info,
            rsp_node,
            gz_sim_launch,
            gz_spawn_entity,
            start_js_after_spawn,
            start_arm_after_js_exit,
        ]
    )

    return ld

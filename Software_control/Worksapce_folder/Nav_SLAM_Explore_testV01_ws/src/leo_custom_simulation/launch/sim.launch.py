import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_custom = get_package_share_directory("leo_custom_simulation")
    pkg_worlds = get_package_share_directory("leo_custom_simulation")

    sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value=os.path.join(pkg_worlds, "worlds", "leo_empty.sdf"),
        description="Path to the Gazebo world file",
    )

    robot_ns = DeclareLaunchArgument("robot_ns", default_value="", description="Robot namespace")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": LaunchConfiguration("sim_world")}.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_custom, "launch", "spawn_with_lidar.launch.py")),
        launch_arguments={"robot_ns": LaunchConfiguration("robot_ns")}.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"qos_overrides./tf_static.publisher.durability": "transient_local"}],
        output="screen",
    )

    return LaunchDescription([sim_world, robot_ns, gz_sim, spawn_robot, clock_bridge])

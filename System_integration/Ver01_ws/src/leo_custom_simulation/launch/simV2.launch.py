import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_directory = get_package_share_directory("leo_custom_simulation")
    meshes_dir = os.path.join(pkg_directory, "meshes")
    pkg_share_parent = os.path.dirname(pkg_directory)
    
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_sim_resource_path = os.pathsep.join(
        [p for p in [existing, pkg_share_parent, meshes_dir] if p]
    )

    sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value=os.path.join(pkg_directory, "worlds", "testspace01.sdf"),
        description="Path to the Gazebo world file",
    )

    robot_ns = DeclareLaunchArgument("robot_ns", default_value="", description="Robot namespace")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": LaunchConfiguration("sim_world")}.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_directory, "launch", "spawn_with_lidar.launch.py")),
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

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=gz_sim_resource_path,
            ),
            sim_world,
            robot_ns,
            gz_sim,
            spawn_robot,
            clock_bridge,
        ]
    )

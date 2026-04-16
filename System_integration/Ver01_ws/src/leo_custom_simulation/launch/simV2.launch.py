import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_launch_module(module_name: str, module_path: str):
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load launch file: {module_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _create_scan_bridge(context, namespace):
    robot_ns = context.perform_substitution(namespace).strip("/")
    scan_topic = "/scan" if robot_ns == "" else f"/{robot_ns}/scan"
    node_name = "scan_bridge" if robot_ns == "" else f"{robot_ns}_scan_bridge"

    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=node_name,
            arguments=[f"{scan_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
            output="screen",
        )
    ]


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_custom = get_package_share_directory("leo_custom_simulation")
    meshes_dir = os.path.join(pkg_custom, "meshes")
    pkg_share_parent = os.path.dirname(pkg_custom)
    gz_sim_resource_path = pkg_share_parent + os.pathsep + meshes_dir

    official_spawn_path = "/opt/ros/jazzy/share/leo_gz_bringup/launch/spawn_robot.launch.py"
    custom_xacro_path = os.path.join(
        pkg_custom, "urdf", "leo_sim_with_lidar_v2.urdf.xacro"
    )

    official_spawn_module = _load_launch_module(
        "leo_gz_spawn_robot_official", official_spawn_path
    )
    original_xacro_process = official_spawn_module.xacro.process

    # Reuse the official spawn logic, but swap in the custom robot description.
    def _process_custom_robot_description(_input_path, mappings=None, **kwargs):
        launch_mappings = {} if mappings is None else dict(mappings)
        return original_xacro_process(
            custom_xacro_path,
            mappings=launch_mappings,
            **kwargs,
        )

    official_spawn_module.xacro.process = _process_custom_robot_description

    sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value=os.path.join(pkg_custom, "worlds", "testspace01.sdf"),
        description="Path to the Gazebo world file",
    )

    robot_ns = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": LaunchConfiguration("sim_world")}.items(),
    )

    spawn_robot = OpaqueFunction(
        function=official_spawn_module.spawn_robot,
        args=[LaunchConfiguration("robot_ns")],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[
            {"qos_overrides./tf_static.publisher.durability": "transient_local"}
        ],
        output="screen",
    )

    scan_bridge = OpaqueFunction(
        function=_create_scan_bridge,
        args=[LaunchConfiguration("robot_ns")],
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
            scan_bridge,
        ]
    )

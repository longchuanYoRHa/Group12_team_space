# mycobot280_pi.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package paths
    pkg_ros_gz = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
    pkg_mycobot = FindPackageShare("mycobot280_pi").find("mycobot280_pi")

    # --- Gazebo (gz_sim) with empty world (from your mycobot_280_gazebo.launch.py) :contentReference[oaicite:0]{index=0}
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_ros_gz, "launch", "gz_sim.launch.py"])]
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # URDF path used both for Gazebo spawn and robot_state_publisher
    urdf_path = os.path.join(pkg_mycobot, "urdf", "mycobot_280_gazebo.urdf")

    # --- Spawn robot into Gazebo (same as before) :contentReference[oaicite:1]{index=1}
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-file", urdf_path, "-name", "mycobot_280"],
        output="screen",
    )

    # --- Robot State Publisher (from your world launch, but without hard-coded path) :contentReference[oaicite:2]{index=2}
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": open(urdf_path).read()
        }],
        output="screen",
    )

    # --- Controllers (from mycobot_280_world.launch.py) :contentReference[oaicite:3]{index=3}
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # --- /clock bridge (from gz_clock_bridge.launch.py, with a small bracket fix) :contentReference[oaicite:4]{index=4}
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        spawn,
        robot_state_publisher,
        joint_state_broadcaster,
        joint_trajectory_controller,
        clock_bridge,
    ])

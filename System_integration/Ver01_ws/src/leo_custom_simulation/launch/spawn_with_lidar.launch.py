import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def spawn_robot(context: LaunchContext, namespace: LaunchConfiguration):
    robot_ns = context.perform_substitution(namespace)

    pkg_custom = get_package_share_directory("leo_custom_simulation")

    robot_desc = xacro.process(
        os.path.join(pkg_custom, "urdf", "leo_with_lidar_camera.urdf.xacro"),
        #change to leo_with_lidar.urdf.xacro if real camera is used
        mappings={"robot_ns": robot_ns},
    )

    robot_gazebo_name = "leo_rover" if robot_ns == "" else "leo_rover_" + robot_ns
    node_name_prefix = "" if robot_ns == "" else robot_ns + "_"

    robot_state_publisher = Node(
        namespace=robot_ns,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, {"robot_description": robot_desc}],
    )

    leo_rover = Node(
        namespace=robot_ns,
        package="ros_gz_sim",
        executable="create",
        name="ros_gz_sim_create",
        output="both",
        arguments=["-topic", "robot_description", "-name", robot_gazebo_name, "-z", "0.15"],
    )

    # D435i: Gazebo topics from leo_with_lidar_camera.urdf.xacro; ROS side matches rover_vision_sim_node (default: /D435i_camera/...)
    d435i_color_info = "D435i_camera/color/camera_info"
    d435i_color_image = "D435i_camera/color/image_raw"
    d435i_depth_image = "D435i_camera/depth/image_raw"

    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=node_name_prefix + "parameter_bridge",
        arguments=[
            robot_ns + "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            robot_ns + "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            robot_ns + "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            robot_ns + "/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU",
            # robot_ns + "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            robot_ns + "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            robot_ns + "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # robot_ns + "/boxes@sensor_msgs/msg/BoundingBoxes[gz.msgs.BoundingBoxes",
            # D435i for rover_vision_sim_node: color camera_info
            d435i_color_info + "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        parameters=[{"qos_overrides./tf_static.publisher.durability": "transient_local"}],
        output="screen",
    )

    # Image bridge: leo camera + D435i color & depth for rover_vision_sim_node
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name=node_name_prefix + "image_bridge",
        arguments=[
            # robot_ns + "/camera/image_raw",
            d435i_color_image,
            d435i_depth_image,
        ],
        output="screen",
    )

    return [robot_state_publisher, leo_rover, topic_bridge, image_bridge]

def generate_launch_description():
    name_argument = DeclareLaunchArgument("robot_ns", default_value="", description="Robot namespace")
    namespace = LaunchConfiguration("robot_ns")
    return LaunchDescription([name_argument, OpaqueFunction(function=spawn_robot, args=[namespace])])

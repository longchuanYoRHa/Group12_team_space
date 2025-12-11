from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    example_nav_pkg_dir = FindPackageShare('example_nav_pkg')
    explore_lite_launch = PathJoinSubstitution(
        [FindPackageShare('explore_lite'), 'launch', 'explore.launch.py']
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    nav = LaunchConfiguration('nav')
    headless = LaunchConfiguration('headless')
    slam_params_file = PathJoinSubstitution(
        [example_nav_pkg_dir, 'config', 'slam_params.yaml']
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='True', 
        description='Use simulation (Gazebo) clock if true'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Whether to run SLAM'
    )

    declare_nav_cmd = DeclareLaunchArgument(
        'nav',
        default_value='True',
        description='Whether to run navigation'
    )

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to run Gazebo in headless mode'
    )

    # Launch TB3 simulation with Nav2 using modified URDF/SDF from this package
    tb3_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([example_nav_pkg_dir, 'tb3_simulation_launch.py'])
        ),
        launch_arguments={
            'slam': slam,  # Use slam parameter to control mapping mode
            'nav': nav,
            'headless': headless,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Launch slam_toolbox node (only if slam is True)
    # This will create the map from scratch in mapping mode
    # The scan is already limited to forward 90 degrees in the URDF
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',  # Standard name for Nav2 compatibility
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(slam),
    )

    # Launch explore node
    explore_lite_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([explore_lite_launch]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_slam_cmd,
        declare_nav_cmd,
        declare_headless_cmd,
        tb3_simulation_launch,
        slam_toolbox_node,
        explore_lite_launch_cmd,
    ])


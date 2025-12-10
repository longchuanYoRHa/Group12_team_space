from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    explore_lite_launch = PathJoinSubstitution(
        [FindPackageShare('explore_lite'), 'launch', 'explore.launch.py']
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    nav = LaunchConfiguration('nav')
    headless = LaunchConfiguration('headless')

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

    # Launch TB3 simulation with Nav2
    tb3_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py'])
        ),
        launch_arguments={
            'slam': slam,
            'nav': nav,
            'headless': headless,
            'use_sim_time': use_sim_time,
        }.items(),
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
        explore_lite_launch_cmd,
    ])


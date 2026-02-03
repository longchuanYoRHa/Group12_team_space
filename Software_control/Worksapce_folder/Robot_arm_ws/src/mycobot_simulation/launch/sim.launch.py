from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    启动 Gazebo 仿真，使用指定的 mycobot_280_pi_adaptive_gripper.urdf 文件
    """
    # Launch 参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    world_file = LaunchConfiguration('world_file')
    
    # 声明 Launch 参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='mycobot_280_pi',
        description='The name for the robot'
    )
    
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value='empty.world',
        description='World file name (empty.world, house.world, pick_and_place_demo.world)'
    )

    # 获取包路径
    description_pkg = get_package_share_directory('mycobot_description')
    gazebo_pkg = get_package_share_directory('mycobot_gazebo')
    
    # URDF 文件路径 - 使用指定的 mycobot_280_pi_adaptive_gripper.urdf
    urdf_file_path = os.path.join(
        description_pkg,
        'urdf',
        'mycobot_280_pi',
        'mycobot_280_pi_adaptive_gripper.urdf'
    )
    
    # 读取 URDF 文件内容
    # 注意：由于该文件包含 xacro 语法，使用 xacro 命令处理
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file_path]),
        value_type=str
    )

    # Robot State Publisher - 发布 robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # Joint State Publisher - 发布关节状态（用于仿真）
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 设置 Gazebo 模型路径
    gazebo_models_path = os.path.join(gazebo_pkg, 'models')
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gazebo_models_path
    )

    # 获取 world 文件路径
    world_path = PathJoinSubstitution([
        gazebo_pkg,
        'worlds',
        world_file
    ])

    # 启动 Gazebo
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[('gz_args', [' -r -v 4 ', world_path])]
    )

    # ROS-Gazebo 桥接配置
    ros_gz_bridge_config_file = os.path.join(
        gazebo_pkg, 'config', 'ros_gz_bridge.yaml'
    )
    
    # ROS-Gazebo 桥接节点
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': ros_gz_bridge_config_file,
        }],
        output='screen'
    )

    # 图像桥接（如果需要相机）
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/camera_head/depth_image',
            '/camera_head/image',
        ],
        remappings=[
            ('/camera_head/depth_image', '/camera_head/depth/image_rect_raw'),
            ('/camera_head/image', '/camera_head/color/image_raw'),
        ],
        output='screen'
    )

    # 在 Gazebo 中生成机器人
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ]
    )

    return LaunchDescription([
        # 声明参数
        declare_use_sim_time,
        declare_robot_name,
        declare_world_file,
        
        # 环境变量
        set_env_vars_resources,
        
        # 机器人描述发布
        robot_state_publisher_node,
        joint_state_publisher_node,
        
        # Gazebo 相关
        start_gazebo_cmd,
        start_gazebo_ros_bridge_cmd,
        start_gazebo_ros_image_bridge_cmd,
        start_gazebo_ros_spawner_cmd,
    ])

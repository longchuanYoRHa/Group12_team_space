# mycobot_simulation/launch/gazebo_sim.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    启动完整的 Gazebo 仿真环境，包括：
    - Robot State Publisher
    - Gazebo 仿真
    - ROS 2 Control 控制器
    """
    # Launch 参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    world_file = LaunchConfiguration('world_file')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    
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
        description='World file name'
    )
    
    declare_x = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Initial x position'
    )
    
    declare_y = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Initial y position'
    )
    
    declare_z = DeclareLaunchArgument(
        'z',
        default_value='0.05',
        description='Initial z position'
    )

    # 获取包路径
    simulation_pkg = get_package_share_directory('mycobot_simulation')
    description_pkg = get_package_share_directory('mycobot_description')
    
    # URDF 文件路径 - 使用增强的 Gazebo URDF
    urdf_file_path = os.path.join(
        simulation_pkg,
        'urdf',
        'mycobot_280_pi_gazebo.urdf.xacro'
    )
    
    # 如果增强的 URDF 不存在，使用原始 URDF（需要手动添加插件）
    if not os.path.exists(urdf_file_path):
        urdf_file_path = os.path.join(
            description_pkg,
            'urdf',
            'mycobot_280_pi',
            'mycobot_280_pi_adaptive_gripper.urdf'
        )
    
    # 读取 URDF 文件内容
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file_path]),
        value_type=str
    )

    # Robot State Publisher
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

    # 获取 world 文件路径
    world_path = PathJoinSubstitution([
        simulation_pkg,
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

    # 在 Gazebo 中生成机器人
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ]
    )

    # 加载控制器配置
    controller_config_file = os.path.join(
        simulation_pkg, 'config', 'ros2_controllers.yaml'
    )

    # 启动控制器管理器
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controller_config_file
        ],
        output='screen'
    )

    # 加载控制器（按顺序）
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_action_controller'],
        output='screen'
    )

    # 控制器加载序列：等待机器人生成后加载
    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[load_joint_state_broadcaster]
    )

    load_arm_controller_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller]
        )
    )

    load_gripper_controller_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_arm_controller,
            on_exit=[load_gripper_controller]
        )
    )

    return LaunchDescription([
        # 声明参数
        declare_use_sim_time,
        declare_robot_name,
        declare_world_file,
        declare_x,
        declare_y,
        declare_z,
        
        # 机器人描述发布
        robot_state_publisher_node,
        
        # Gazebo
        start_gazebo_cmd,
        start_gazebo_ros_spawner_cmd,
        
        # 控制器
        controller_manager_node,
        delayed_joint_state_broadcaster,
        load_arm_controller_handler,
        load_gripper_controller_handler,
    ])
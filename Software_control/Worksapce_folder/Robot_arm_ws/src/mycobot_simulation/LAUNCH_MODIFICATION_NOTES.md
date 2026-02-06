# Launch 文件修改说明

## 修改概述

已将 `sim.launch.py` 修改为使用指定的 URDF 文件 `mycobot_280_pi_adaptive_gripper.urdf`，而不是依赖 `mycobot_gazebo` 包的默认配置。

## 主要修改

### 1. 直接加载指定的 URDF 文件

```python
urdf_file_path = os.path.join(
    description_pkg,
    'urdf',
    'mycobot_280_pi',
    'mycobot_280_pi_adaptive_gripper.urdf'
)
```

### 2. 使用 robot_state_publisher 发布 robot_description

```python
robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
        'robot_description': robot_description_content,
        'use_sim_time': use_sim_time
    }]
)
```

### 3. 启动 Gazebo 仿真环境

- 启动 Gazebo 服务器
- 启动 ROS-Gazebo 桥接
- 启动图像桥接（用于相机数据）
- 在 Gazebo 中生成机器人模型

## 新增的 Launch 参数

- `robot_name`: 机器人名称（默认: `mycobot_280_pi`）
- `world_file`: Gazebo 世界文件（默认: `empty.world`）
- `use_sim_time`: 使用仿真时间（默认: `true`）

## 使用方法

### 基本使用

```bash
ros2 launch mycobot_simulation sim.launch.py
```

### 指定世界文件

```bash
ros2 launch mycobot_simulation sim.launch.py world_file:=pick_and_place_demo.world
```

### 指定机器人名称

```bash
ros2 launch mycobot_simulation sim.launch.py robot_name:=mycobot_280_pi
```

## 依赖关系

该 launch 文件需要以下包：

1. **mycobot_description** - 提供 URDF 文件
2. **mycobot_gazebo** - 提供世界文件和配置文件
3. **ros_gz_sim** - Gazebo 仿真
4. **ros_gz_bridge** - ROS-Gazebo 桥接
5. **ros_gz_image** - 图像桥接

## 注意事项

1. **URDF 文件处理**: 由于 URDF 文件声明了 xacro 命名空间，使用 `xacro` 命令处理文件
2. **包路径**: 确保 `mycobot_description` 和 `mycobot_gazebo` 包已正确安装
3. **世界文件**: 默认使用 `empty.world`，可以更改为其他可用的世界文件
4. **机器人位置**: 机器人在 Gazebo 中的初始位置设置为 (0, 0, 0.05)

## 与原版本的区别

### 原版本
- 依赖 `mycobot_gazebo` 包的 launch 文件
- 使用默认的机器人配置

### 新版本
- 直接指定 URDF 文件路径
- 独立控制机器人描述发布
- 更灵活的配置选项

## 故障排除

### 问题：找不到 URDF 文件

**解决方案**: 确保 `mycobot_description` 包已正确构建和安装

```bash
cd ~/your_ws
colcon build --packages-select mycobot_description
source install/setup.bash
```

### 问题：Gazebo 无法生成机器人

**检查**:
1. `/robot_description` topic 是否发布
   ```bash
   ros2 topic echo /robot_description
   ```
2. Gazebo 是否正常运行
3. ROS-Gazebo 桥接是否启动

### 问题：xacro 命令失败

如果 URDF 文件不需要 xacro 处理，可以修改为直接读取：

```python
# 直接读取文件内容
with open(urdf_file_path, 'r') as infile:
    robot_description_content = infile.read()
    
robot_description_content = ParameterValue(
    robot_description_content,
    value_type=str
)
```

## 下一步

如果需要添加更多功能，可以考虑：

1. 添加控制器加载（ROS 2 Control）
2. 添加 MoveIt 2 集成
3. 添加 RViz 可视化
4. 添加相机配置选项


# TF2 快速使用指南 / TF2 Quick Start Guide

## 🚀 快速开始 / Quick Start

### 1. 安装依赖 / Install Dependencies

```bash
# 安装 TF2 相关包
sudo apt update
sudo apt install -y \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf-transformations

# 安装 Python 依赖
pip install transforms3d
```

### 2. 编译新代码 / Build New Code

```bash
cd ~/mycobot_ws
colcon build --packages-select my_cobot_control
source install/setup.bash
```

### 3. 启动 TF2 版本 / Launch TF2 Version

```bash
# 使用默认配置启动
ros2 launch my_cobot_control mycobot_with_tf2.launch.py

# 自定义摄像头位置
ros2 launch my_cobot_control mycobot_with_tf2.launch.py \
    camera_x:=0.25 \
    camera_y:=-0.15 \
    camera_z:=0.35 \
    camera_pitch:=-45.0
```

### 4. 测试 TF2 变换 / Test TF2 Transforms

```bash
# 在另一个终端运行测试
cd ~/mycobot_ws/src/my_cobot_control/scripts
python3 test_tf2_transform.py
```

### 5. 发送测试目标 / Send Test Target

```bash
# 发送 pick 命令（摄像头坐标系）
ros2 topic pub --once /arm/target_pick geometry_msgs/msg/Point \
  "{x: 100.0, y: 50.0, z: 80.0}"

# 观察日志，会显示：
# [PICK] Camera coords: (100.0, 50.0, 80.0) mm
# [PICK] Base/Flange:   (XXX, YYY, ZZZ) mm
```

---

## 📊 对比：当前方法 vs TF2 方法

### 方法对比表

| 特性 | 当前方法（静态参数） | TF2 方法 |
|------|---------------------|----------|
| **固定摄像头** | ✅ 简单高效 | ✅ 也可以用 |
| **移动小车** | ❌ 无法处理 | ✅ **完美支持** |
| **多级坐标变换** | ❌ 需手动计算 | ✅ 自动处理 |
| **Gripper 偏移** | ❌ 硬编码 | ✅ TF2 自动 |
| **可视化调试** | ❌ 困难 | ✅ RViz2 |
| **多传感器融合** | ❌ 每个单独处理 | ✅ 统一框架 |
| **时间同步** | ❌ 无 | ✅ 支持 |
| **配置复杂度** | 简单 | 中等 |
| **计算开销** | 很小 | 小 |

### 推荐使用场景

**使用当前方法（静态参数）**：
- 摄像头和机械臂都固定安装
- 快速原型开发
- 不需要可视化调试

**使用 TF2 方法**：
- ✅ **小车会移动**（强烈推荐！）
- ✅ 需要精确的 gripper 偏移补偿
- ✅ 使用多个传感器
- ✅ 需要可视化调试
- ✅ 与导航/SLAM 集成

---

## 🔧 配置 TF2 坐标系树

### Launch 文件配置

编辑 `launch/mycobot_with_tf2.launch.py` 修改坐标系关系：

```python
# 摄像头相对于小车底盘的位置
camera_tf_broadcaster = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '0.2',    # X: 小车前方 200mm
        '-0.1',   # Y: 左侧 100mm
        '0.3',    # Z: 上方 300mm
        '0',      # Roll
        '-0.524', # Pitch: 向下倾斜 30度 (-30° = -0.524 rad)
        '0',      # Yaw
        'rover_base',
        'camera_link'
    ]
)
```

### Gripper 偏移配置

```python
# Gripper base 偏移（从 URDF）
gripper_base_tf_broadcaster = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '0.0', '0.0', '0.034',  # +34mm Z
        '1.579', '0', '0',      # 旋转 90度
        'joint6_flange',
        'gripper_base'
    ]
)

# Gripper tip 偏移（手指长度）
gripper_tip_tf_broadcaster = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '0.0', '0.0', '0.045',  # +45mm Z
        '0', '0', '0',
        'gripper_base',
        'gripper_tip'
    ]
)
```

---

## 🎯 关于 Gripper 偏移的重要说明

### `send_coords` 控制什么？

**`send_coords(x, y, z, rx, ry, rz)` 控制的是 `joint6_flange` 的位置，不是夹爪末端！**

```
          ┌───────────┐
          │ 夹爪末端   │ ← gripper_tip (+79mm from flange)
          │ gripper_tip│
          └───────────┘
               ││
          ┌───────────┐
          │ 夹爪基座   │ ← gripper_base (+34mm from flange)
          └───────────┘
               ││
          ┌───────────┐
          │ 法兰盘中心 │ ← joint6_flange (send_coords 控制这里)
          │   flange   │
          └───────────┘
```

### TF2 如何自动处理 Gripper 偏移？

#### 方法 1: 自动从 TF2 查询偏移量（推荐）

```python
# 在 mycobot_controller_tf2.py 中
def _get_gripper_offset_from_tf2(self):
    transform = self.tf_buffer.lookup_transform(
        target_frame='gripper_tip',
        source_frame='joint6_flange',
        time=rclpy.time.Time()
    )
    # 返回 Z 偏移
    return transform.transform.translation.z * 1000.0  # 约 79mm
```

TF2 会自动查找：`joint6_flange -> gripper_base -> gripper_tip`

#### 方法 2: 直接转换到 gripper_tip

```python
# 知道物体相对于 gripper_tip 的位置
x_tip, y_tip, z_tip = self._transform_point_tf2(
    x_cam, y_cam, z_cam,
    target_frame='gripper_tip',
    source_frame='camera_link'
)

# 然后计算 flange 需要到达的位置
z_flange = z_tip - gripper_offset
```

---

## 📊 TF2 坐标系树结构

### 完整坐标系层级

```
world (全局固定参考系)
  │
  └── rover_base (小车底盘，由导航系统更新)
        │
        ├── camera_link (摄像头，固定在小车上)
        │
        ├── lidar_link (激光雷达，可选)
        │
        └── arm_base_link (机械臂基座，固定在小车上)
              │
              └── joint1 (由机械臂 URDF 或状态发布器更新)
                    └── joint2
                          └── joint3
                                └── joint4
                                      └── joint5
                                            └── joint6
                                                  └── joint6_flange ← send_coords 控制
                                                        │
                                                        └── gripper_base (+34mm)
                                                              │
                                                              ├── gripper_left_finger
                                                              ├── gripper_right_finger
                                                              └── gripper_tip (+45mm)
```

### 关键变换

| 变换 | 类型 | 更新方式 | 说明 |
|------|------|----------|------|
| `world -> rover_base` | 动态 | 导航系统 | 小车在世界中的位置 |
| `rover_base -> camera_link` | 静态 | static_transform_publisher | 摄像头安装位置 |
| `rover_base -> arm_base_link` | 静态 | static_transform_publisher | 机械臂安装位置 |
| `arm_base_link -> joint1` | 固定 | URDF 或状态发布器 | 机械臂运动学 |
| `joint6_flange -> gripper_base` | 静态 | static_transform_publisher | 夹爪基座偏移 |
| `gripper_base -> gripper_tip` | 静态 | static_transform_publisher | 夹爪手指长度 |

---

## 🔍 调试和可视化工具

### 1. 查看 TF 树

```bash
# 生成 TF 树的 PDF 文件
ros2 run tf2_tools view_frames

# 查看生成的 frames.pdf
evince frames.pdf
```

### 2. 实时监听变换

```bash
# 监听 camera_link 到 arm_base_link 的变换
ros2 run tf2_ros tf2_echo camera_link arm_base_link

# 输出示例:
# At time 1234.56
# - Translation: [0.200, -0.100, 0.300]  (米)
# - Rotation: in RPY (degree) [0.000, -30.000, 0.000]
```

### 3. 在 RViz2 中可视化

```bash
# 启动 RViz2
rviz2

# 在 RViz2 中:
# 1. 点击 "Add" -> "TF" -> "OK"
# 2. 设置 "Fixed Frame" 为 "arm_base_link" 或 "rover_base"
# 3. 可以看到所有坐标系的箭头和连接线
# 4. 调整 "TF" -> "Show Names" 显示坐标系名称
```

### 4. 列出所有坐标系

```bash
# 列出当前系统中的所有坐标系
ros2 run tf2_ros tf2_monitor

# 或者
ros2 topic echo /tf_static
```

---

## 🧪 测试流程

### 完整测试步骤

```bash
# 终端 1: 启动 TF2 系统
ros2 launch my_cobot_control mycobot_with_tf2.launch.py

# 终端 2: 运行测试脚本
cd ~/mycobot_ws/src/my_cobot_control/scripts
python3 test_tf2_transform.py

# 终端 3: 可视化（可选）
rviz2

# 终端 4: 发送测试目标
ros2 topic pub --once /arm/target_pick geometry_msgs/msg/Point \
  "{x: 100.0, y: 50.0, z: 80.0}"
```

### 预期输出

```
[INFO] [PICK] Camera coords: (100.0, 50.0, 80.0) mm
[INFO]   Gripper offset from TF2: 79.0 mm
[INFO] [PICK] Base/Flange:   (285.3, -42.1, 301.2) mm
[INFO] [PICK 1/6] Open gripper
[INFO] [PICK 2/6] Move above pick: [285.3, -42.1, 250.0, ...]
...
```

---

## ❓ 常见问题 / FAQ

### Q1: TF2 lookup 失败，提示 "Frame does not exist"

**原因**: TF2 broadcaster 还没有启动或坐标系名称错误

**解决**:
```bash
# 检查当前可用的坐标系
ros2 topic echo /tf_static

# 确认 static_transform_publisher 节点在运行
ros2 node list | grep tf
```

### Q2: 坐标转换后位置不正确

**原因**: 参数配置错误（单位、角度、正负号）

**解决**:
```bash
# 查看实际的变换
ros2 run tf2_ros tf2_echo camera_link arm_base_link

# 检查配置:
# - 平移单位是米 (m)，不是毫米
# - 旋转单位是弧度 (rad)，不是度
# - 检查旋转方向（正负号）
```

### Q3: 需要在运行时动态调整 TF2 变换

**解决**: 使用参数化的 broadcaster

```python
# 创建自定义节点发布动态变换
from tf2_ros import TransformBroadcaster

broadcaster = TransformBroadcaster(self)
# 定期更新变换...
```

### Q4: 如何从现有方法迁移到 TF2？

**逐步迁移**:

1. **第一步**: 保持当前代码，添加 TF2 broadcaster
2. **第二步**: 验证 TF2 变换与手动计算一致
3. **第三步**: 切换到 TF2 版本控制器
4. **第四步**: 移除旧的静态参数代码

---

## 📚 进一步学习 / Further Reading

- **TF2 官方教程**: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2.html
- **TF2 设计文档**: http://wiki.ros.org/tf2/Design
- **坐标变换数学**: https://en.wikipedia.org/wiki/Transformation_matrix

---

## 📝 总结 / Summary

### TF2 核心优势

1. **自动处理多级变换** - 不需要手动计算复杂的坐标转换链
2. **支持动态坐标系** - 小车移动时自动更新所有相关坐标
3. **精确的 gripper 偏移** - 通过 TF2 树自动计算法兰盘到夹爪末端的偏移
4. **强大的调试工具** - RViz2 可视化、tf2_echo 监听、view_frames 查看树结构
5. **标准化接口** - 与其他 ROS2 节点（导航、SLAM、MoveIt）无缝集成

### 下一步行动

✅ 安装 TF2 依赖  
✅ 编译并测试 TF2 版本  
✅ 在 RViz2 中可视化坐标系树  
✅ 根据实际安装位置调整 launch 文件参数  
✅ 运行完整的 pick-and-place 测试  

---

**需要帮助？** 查看详细指南：[TF2_TRANSFORM_GUIDE.md](../../TF2_TRANSFORM_GUIDE.md)

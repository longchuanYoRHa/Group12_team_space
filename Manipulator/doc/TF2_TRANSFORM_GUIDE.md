# ROS2 TF2 坐标变换详解 / TF2 Transform Guide

## 什么是 TF2？ / What is TF2?

**TF2 (Transform Library 2)** 是 ROS2 中用于管理坐标系变换的核心库。它维护一个**坐标系树（transform tree）**，自动处理多个坐标系之间的变换关系。

### 核心概念

```
        world (固定参考系)
          |
          ├─── rover_base (小车底盘)
          |      |
          |      ├─── camera_link (摄像头)
          |      |
          |      └─── arm_base_link (机械臂基座)
          |             |
          |             ├─── joint1
          |             ├─── joint2
          |             ├─── ...
          |             ├─── joint6
          |             ├─── joint6_flange (法兰盘 - send_coords 控制点)
          |             └─── gripper_base
          |                    |
          |                    └─── gripper_tip (夹爪末端)
```

---

## 当前方法 vs TF2 方法 / Current Method vs TF2

### 当前方法：静态参数转换

**优点**：
- ✅ 简单直接
- ✅ 不需要额外的节点
- ✅ 计算开销小
- ✅ 适合固定安装的摄像头

**缺点**：
- ❌ 硬编码的转换关系
- ❌ 无法处理动态变换（如小车移动时）
- ❌ 需要手动计算多级变换
- ❌ 无法与其他 ROS2 节点共享变换信息
- ❌ 调试困难，无法可视化

**代码实现**：
```python
# 硬编码的转换矩阵
def _transform_camera_to_base(self, x_cam, y_cam, z_cam):
    tx, ty, tz = self._camera_to_base[:3]
    rx, ry, rz = self._camera_to_base[3:]
    # 手动计算旋转矩阵...
    R = Rz @ Ry @ Rx
    p_base = R @ p_cam + t
    return p_base
```

### TF2 方法：动态坐标系树

**优点**：
- ✅ **动态变换**：支持移动的坐标系（如小车移动）
- ✅ **多级变换**：自动查找并组合变换链
- ✅ **时间同步**：可以查询历史变换，处理传感器延迟
- ✅ **可视化**：可用 RViz2 查看坐标系关系
- ✅ **标准接口**：其他 ROS2 节点可共享变换
- ✅ **自动插值**：在变换之间自动插值

**缺点**：
- ❌ 需要额外的 broadcaster 节点
- ❌ 稍微复杂一些
- ❌ 有轻微的查询开销

**代码实现**：
```python
# TF2 自动查询变换
def _transform_camera_to_base(self, x_cam, y_cam, z_cam):
    # TF2 自动查找 camera_link -> arm_base_link 的完整变换链
    transform = self.tf_buffer.lookup_transform(
        'arm_base_link', 'camera_link', rclpy.time.Time())
    # 应用变换
    point_base = tf2_geometry_msgs.do_transform_point(point_cam, transform)
    return point_base
```

---

## TF2 的关键优势场景

### 场景 1: 小车移动时的动态变换

**问题**：小车移动时，摄像头相对于世界坐标系的位置在变化。

**当前方法**：❌ 无法处理，因为 `camera_to_base` 是固定的

**TF2 方法**：✅ 自动更新
```
world -> rover_base (动态，由导航系统更新)
rover_base -> camera_link (固定)
rover_base -> arm_base_link (固定)

TF2 自动计算: world -> camera_link -> arm_base_link
```

### 场景 2: 考虑 Gripper 偏移

**问题**：`send_coords` 控制的是 joint6_flange，但我们想知道 gripper_tip 的位置。

**当前方法**：❌ 需要手动计算偏移
```python
# 硬编码
z_target = z_camera - 80  # gripper_tip 偏移约 80mm
```

**TF2 方法**：✅ 自动处理
```python
# TF2 定义坐标系树
joint6_flange -> gripper_base (+34mm Z)
gripper_base -> gripper_tip (+45mm Z)

# 查询时自动计算完整变换
transform = tf_buffer.lookup_transform(
    'gripper_tip', 'camera_link', rclpy.time.Time())
```

### 场景 3: 多个传感器融合

**问题**：同时使用摄像头、激光雷达、IMU 等多个传感器。

**当前方法**：❌ 需要为每个传感器写单独的转换函数

**TF2 方法**：✅ 统一的坐标系树
```
world
├── rover_base
    ├── camera_link
    ├── lidar_link
    ├── imu_link
    └── arm_base_link
```

---

## TF2 使用步骤 / How to Use TF2

### Step 1: 广播静态变换 (Static Transform Broadcaster)

创建一个节点或在 launch 文件中广播固定的坐标系关系。

#### 方法 A: 使用 launch 文件（推荐）

```python
# launch/mycobot_with_tf.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 广播 rover_base -> camera_link 的变换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_broadcaster',
            arguments=[
                '0.2', '-0.1', '0.3',    # x, y, z (单位: 米)
                '0', '-0.524', '0',      # roll, pitch, yaw (单位: 弧度, -30度 = -0.524)
                'rover_base',            # 父坐标系
                'camera_link'            # 子坐标系
            ]
        ),
        
        # 广播 rover_base -> arm_base_link 的变换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='arm_tf_broadcaster',
            arguments=[
                '0.0', '0.0', '0.05',    # 机械臂距离小车底盘 50mm
                '0', '0', '0',
                'rover_base',
                'arm_base_link'
            ]
        ),
        
        # 广播 joint6_flange -> gripper_base 的变换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gripper_base_tf_broadcaster',
            arguments=[
                '0.0', '0.0', '0.034',   # +34mm Z
                '1.579', '0', '0',       # 旋转约 90 度
                'joint6_flange',
                'gripper_base'
            ]
        ),
        
        # 广播 gripper_base -> gripper_tip 的变换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gripper_tip_tf_broadcaster',
            arguments=[
                '0.0', '0.0', '0.045',   # +45mm Z (手指长度)
                '0', '0', '0',
                'gripper_base',
                'gripper_tip'
            ]
        ),
        
        # 启动机械臂控制器
        Node(
            package='my_cobot_control',
            executable='mycobot_controller_tf2',
            name='mycobot_controller',
            namespace='arm',
            output='screen'
        ),
    ])
```

#### 方法 B: 在代码中广播

```python
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot')
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 广播 rover_base -> camera_link
        self._broadcast_static_transform(
            parent='rover_base',
            child='camera_link',
            xyz=[0.2, -0.1, 0.3],
            rpy=[0, -0.524, 0]
        )
    
    def _broadcast_static_transform(self, parent, child, xyz, rpy):
        from tf_transformations import quaternion_from_euler
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        
        t.transform.translation.x = xyz[0]
        t.transform.translation.y = xyz[1]
        t.transform.translation.z = xyz[2]
        
        quat = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.static_tf_broadcaster.sendTransform(t)
```

### Step 2: 在代码中查询变换

```python
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

class MyCobotControllerTF2(Node):
    def __init__(self):
        super().__init__('mycobot_controller_tf2')
        
        # 创建 TF2 buffer 和 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 等待变换可用
        self.get_logger().info('Waiting for TF transforms...')
    
    def transform_camera_to_base(self, x_cam, y_cam, z_cam):
        """
        使用 TF2 将摄像头坐标转换为机械臂基座坐标。
        
        Args:
            x_cam, y_cam, z_cam: 摄像头坐标系中的点 (mm)
        Returns:
            (x_base, y_base, z_base): 机械臂基座坐标系中的点 (mm)
        """
        try:
            # 创建点（转换为米）
            point_cam = PointStamped()
            point_cam.header.frame_id = 'camera_link'
            point_cam.header.stamp = self.get_clock().now().to_msg()
            point_cam.point.x = x_cam / 1000.0  # mm -> m
            point_cam.point.y = y_cam / 1000.0
            point_cam.point.z = z_cam / 1000.0
            
            # 查询变换（TF2 会自动查找完整的变换链）
            transform = self.tf_buffer.lookup_transform(
                target_frame='arm_base_link',      # 目标坐标系
                source_frame='camera_link',        # 源坐标系
                time=rclpy.time.Time(),            # 最新的变换
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 应用变换
            point_base = tf2_geometry_msgs.do_transform_point(point_cam, transform)
            
            # 转换回毫米
            return (
                point_base.point.x * 1000.0,
                point_base.point.y * 1000.0,
                point_base.point.z * 1000.0
            )
            
        except Exception as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            # 失败时返回原始坐标
            return x_cam, y_cam, z_cam
    
    def transform_camera_to_gripper_tip(self, x_cam, y_cam, z_cam):
        """
        直接将摄像头坐标转换为夹爪末端坐标（考虑了所有偏移）。
        
        这样你就知道夹爪末端需要移动到哪里！
        """
        try:
            point_cam = PointStamped()
            point_cam.header.frame_id = 'camera_link'
            point_cam.header.stamp = self.get_clock().now().to_msg()
            point_cam.point.x = x_cam / 1000.0
            point_cam.point.y = y_cam / 1000.0
            point_cam.point.z = z_cam / 1000.0
            
            # 直接查询到 gripper_tip 的变换
            # TF2 会自动计算: camera_link -> arm_base_link -> ... -> gripper_tip
            transform = self.tf_buffer.lookup_transform(
                target_frame='gripper_tip',
                source_frame='camera_link',
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            point_gripper = tf2_geometry_msgs.do_transform_point(point_cam, transform)
            
            return (
                point_gripper.point.x * 1000.0,
                point_gripper.point.y * 1000.0,
                point_gripper.point.z * 1000.0
            )
            
        except Exception as e:
            self.get_logger().error(f'TF lookup to gripper_tip failed: {e}')
            return x_cam, y_cam, z_cam
```

### Step 3: 考虑 Gripper 偏移的完整示例

```python
def _pick_cb(self, msg: Point):
    """接收摄像头检测到的目标位置。"""
    
    # 方法 1: 转换到 arm_base_link，然后手动减去 gripper 偏移
    x_base, y_base, z_base = self.transform_camera_to_base(msg.x, msg.y, msg.z)
    
    # send_coords 控制的是 joint6_flange，所以要补偿 gripper 偏移
    GRIPPER_OFFSET = 80.0  # mm (34mm 基座 + 45mm 手指)
    z_flange = z_base - GRIPPER_OFFSET
    
    self.get_logger().info(f'Camera: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})')
    self.get_logger().info(f'Base: ({x_base:.1f}, {y_base:.1f}, {z_base:.1f})')
    self.get_logger().info(f'Flange target: ({x_base:.1f}, {y_base:.1f}, {z_flange:.1f})')
    
    # 发送给机械臂
    self.mc.send_coords([x_base, y_base, z_flange, rx, ry, rz], speed, mode)
    
    # 方法 2: 使用 TF2 直接查询到 gripper_tip 的变换
    # 这样你知道物体相对于 gripper_tip 的位置，可以更精确地规划
    x_tip, y_tip, z_tip = self.transform_camera_to_gripper_tip(msg.x, msg.y, msg.z)
    self.get_logger().info(f'Gripper tip should reach: ({x_tip:.1f}, {y_tip:.1f}, {z_tip:.1f})')
```

---

## 完整的 TF2 坐标系树定义

### 推荐的坐标系层级

```
world (固定世界坐标系)
  |
  └─ rover_base (小车底盘，由导航系统更新位置)
       |
       ├─ camera_link (摄像头，固定在小车上)
       |
       ├─ lidar_link (激光雷达，可选)
       |
       └─ arm_base_link (机械臂基座，固定在小车上)
            |
            └─ joint1 (由机械臂 URDF 或状态发布器更新)
                 └─ joint2
                      └─ joint3
                           └─ joint4
                                └─ joint5
                                     └─ joint6
                                          └─ joint6_flange (send_coords 控制点)
                                               |
                                               └─ gripper_base (+34mm Z, 旋转)
                                                    |
                                                    ├─ gripper_left_finger
                                                    ├─ gripper_right_finger
                                                    └─ gripper_tip (+45mm Z，抓取参考点)
```

---

## 可视化 TF2 树 / Visualize TF Tree

### 1. 查看文本形式的 TF 树

```bash
# 安装工具
sudo apt install ros-humble-tf2-tools

# 查看当前的 TF 树
ros2 run tf2_tools view_frames

# 这会生成 frames.pdf 文件，显示坐标系关系图
```

### 2. 在 RViz2 中查看

```bash
# 启动 RViz2
rviz2

# 添加显示项:
# 1. Add -> TF -> OK  (显示所有坐标系)
# 2. 设置 Fixed Frame 为 "world" 或 "arm_base_link"
# 3. 可以看到所有坐标系的位置和方向
```

### 3. 监听特定变换

```bash
# 实时查看 camera_link 到 arm_base_link 的变换
ros2 run tf2_ros tf2_echo camera_link arm_base_link

# 输出:
# At time 1234.56
# - Translation: [0.200, -0.100, 0.300]
# - Rotation: in Quaternion [0.000, -0.259, 0.000, 0.966]
#            in RPY (radian) [0.000, -0.524, 0.000]
#            in RPY (degree) [0.000, -30.000, 0.000]
```

---

## 安装依赖 / Install Dependencies

```bash
# 安装 TF2 相关包
sudo apt install ros-humble-tf2-ros \
                 ros-humble-tf2-geometry-msgs \
                 ros-humble-tf2-tools \
                 ros-humble-tf-transformations

# 如果使用 Python，还需要安装
pip install transforms3d
```

---

## 总结：何时使用哪种方法？ / When to Use Which Method?

### 使用当前方法（静态参数）的场景：

- ✅ 摄像头和机械臂都固定不动
- ✅ 只需要简单的坐标转换
- ✅ 不需要与其他 ROS2 节点共享变换
- ✅ 快速原型开发

### 使用 TF2 的场景：

- ✅ **小车会移动**（强烈推荐！）
- ✅ 需要考虑多级坐标变换（如 gripper 偏移）
- ✅ 使用多个传感器（摄像头 + 雷达 + IMU）
- ✅ 需要可视化调试坐标系关系
- ✅ 与导航、SLAM 等其他 ROS2 功能集成
- ✅ 需要处理传感器延迟和时间同步

### 混合方法（推荐）：

对于您的小车 + 机械臂系统，我推荐：
1. **使用 TF2 管理所有坐标系关系**（包括 gripper 偏移）
2. **在控制器中查询 TF2** 进行坐标转换
3. **仍然使用参数文件** 配置静态变换的初始值

---

## 下一步 / Next Steps

1. 阅读完整的 TF2 实现代码：`mycobot_controller_tf2.py`
2. 查看 launch 文件示例：`mycobot_with_tf.launch.py`
3. 运行测试脚本验证变换：`test_tf2_transform.py`
4. 在 RViz2 中可视化坐标系树

详细实现请参考接下来创建的代码文件！

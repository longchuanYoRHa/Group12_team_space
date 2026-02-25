# 视觉算法到控制节点的坐标传递指南

## 概述

本文档说明如何将视觉算法检测到的物体坐标（相对于相机坐标系）传递给控制节点，以便计算导航进近目标点。

## 工作流程

```
视觉算法 → 发布 PointStamped (相机坐标系) → obj_xy topic → 控制节点 → 自动转换到 map 坐标系 → 计算进近目标点
```

## 步骤 1: 视觉算法发布坐标

### 1.1 消息格式

视觉算法需要发布 `geometry_msgs/PointStamped` 消息到 `obj_xy` topic。

**消息结构：**
```python
PointStamped:
  header:
    stamp: 时间戳
    frame_id: 相机坐标系名称 (例如: "D435i_camera_color_optical_frame")
  point:
    x: 物体在相机坐标系下的 X 坐标 (米)
    y: 物体在相机坐标系下的 Y 坐标 (米)
    z: 物体在相机坐标系下的 Z 坐标 (米，深度)
```

### 1.2 相机坐标系说明

根据你的 URDF 配置，相机坐标系可能是：
- `D435i_camera_color_optical_frame` - RGB 相机光学坐标系（推荐用于 2D 检测）
- `D435i_camera_depth_optical_frame` - 深度相机光学坐标系（推荐用于 3D 检测）

**坐标系约定（相机光学坐标系）：**
- **X 轴**: 向右为正
- **Y 轴**: 向下为正
- **Z 轴**: 向前为正（深度）

### 1.3 示例代码

参考 `example_vision_publisher.py` 文件，其中包含完整的发布示例。

**关键代码片段：**
```python
from geometry_msgs.msg import PointStamped

# 创建发布器
obj_pub = self.create_publisher(PointStamped, 'obj_xy', 10)

# 从视觉算法获取坐标（相机坐标系）
x, y, z = your_vision_algorithm()  # 单位：米

# 创建消息
point_msg = PointStamped()
point_msg.header.stamp = self.get_clock().now().to_msg()
point_msg.header.frame_id = 'D435i_camera_color_optical_frame'  # 相机坐标系
point_msg.point.x = x
point_msg.point.y = y
point_msg.point.z = z

# 发布
obj_pub.publish(point_msg)
```

## 步骤 2: 控制节点自动处理

控制节点 (`task_manager_node.py`) 会自动：

1. **订阅 `obj_xy` topic** - 接收视觉算法发布的坐标
2. **自动坐标系转换** - 从相机坐标系转换到 map 坐标系
3. **存储坐标** - 保存到 `self.object_pose` 供后续使用
4. **计算进近目标点** - 在 `handle_nav_to_object_pregrasp_state()` 中使用坐标

### 2.1 坐标系转换

控制节点使用 TF2 自动进行坐标系转换：

```python
# 在 obj_xy_callback() 中
transform = self.tf_buffer.lookup_transform(
    'map',                    # 目标坐标系
    msg.header.frame_id,      # 源坐标系（相机坐标系）
    rclpy.time.Time()         # 使用最新变换
)

# 转换点坐标
point_stamped_in_map = tf2_geometry_msgs.do_transform_point(msg, transform)
```

**要求：**
- TF 树必须包含从相机坐标系到 `map` 坐标系的变换链
- 变换链通常为：`map → base_link → camera_link → camera_optical_frame`

### 2.2 坐标使用

转换后的坐标存储在 `self.object_pose` 中，格式为 `PoseStamped`（map 坐标系）。

在 `handle_nav_to_object_pregrasp_state()` 中：
```python
# 检查坐标是否可用
if self.object_pose is None:
    self.get_logger().error('物体位姿不可用！')
    return

# 使用坐标计算预抓取位置
goal_pose = self.calculate_pregrasp_pose(self.object_pose, pregrasp_distance)
```

## 步骤 3: 验证和调试

### 3.1 检查 TF 变换

使用以下命令检查 TF 树：
```bash
ros2 run tf2_tools view_frames
```

或实时查看：
```bash
ros2 run tf2_ros tf2_echo <source_frame> <target_frame>
# 例如：
ros2 run tf2_ros tf2_echo D435i_camera_color_optical_frame map
```

### 3.2 检查消息发布

监听 `obj_xy` topic：
```bash
ros2 topic echo /obj_xy
```

应该看到类似输出：
```
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: 'D435i_camera_color_optical_frame'
point:
  x: 0.5
  y: 0.1
  z: 0.8
```

### 3.3 检查坐标转换

查看控制节点日志，应该看到：
```
接收到物体坐标 (D435i_camera_color_optical_frame坐标系): (0.500, 0.100, 0.800), 
已转换到 map 坐标系: (2.345, 1.234, 0.800)
```

## 常见问题

### Q1: 坐标系转换失败

**错误信息：** `坐标系转换失败: ...`

**解决方案：**
1. 检查 TF 树是否完整：`ros2 run tf2_tools view_frames`
2. 确认相机坐标系名称正确
3. 检查相机节点是否正在发布 TF 变换

### Q2: 坐标不准确

**可能原因：**
1. 相机标定不准确
2. TF 变换链中的变换参数错误
3. 视觉算法坐标计算错误

**解决方案：**
1. 重新标定相机
2. 检查 URDF 中的相机安装位置和姿态
3. 验证视觉算法的坐标计算

### Q3: 消息未接收

**检查项：**
1. 视觉节点是否正在运行
2. topic 名称是否正确 (`obj_xy`)
3. 消息类型是否正确 (`geometry_msgs/PointStamped`)
4. 使用 `ros2 topic list` 和 `ros2 topic echo /obj_xy` 验证

## 完整示例

### 视觉节点 (Python)

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.pub = self.create_publisher(PointStamped, 'obj_xy', 10)
        self.timer = self.create_timer(0.1, self.publish_coords)
    
    def publish_coords(self):
        # 从你的视觉算法获取坐标
        x, y, z = your_vision_algorithm()  # 相机坐标系，单位：米
        
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'D435i_camera_color_optical_frame'
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        
        self.pub.publish(msg)
```

### 控制节点 (已实现)

控制节点会自动：
- 订阅 `obj_xy` topic
- 转换坐标系
- 存储坐标
- 计算进近目标点

## 总结

1. **视觉算法**：发布 `PointStamped` 到 `obj_xy` topic，frame_id 设置为相机坐标系
2. **控制节点**：自动订阅、转换坐标系、存储并使用坐标
3. **坐标系转换**：通过 TF2 自动完成，无需手动计算
4. **进近目标点**：在 `handle_nav_to_object_pregrasp_state()` 中自动计算

只需确保视觉算法正确发布坐标，其余工作由控制节点自动处理！


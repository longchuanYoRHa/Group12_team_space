# 摄像头坐标转换配置指南 / Camera Transform Configuration Guide

## 概述 / Overview

本指南说明如何配置摄像头坐标系到机械臂基座坐标系的转换。

This guide explains how to configure the coordinate transformation from camera frame to robot arm base frame.

---

## 1. `send_coords` 坐标参考点 / Coordinate Reference Point

**重要**: `send_coords(x, y, z, rx, ry, rz)` 控制的是 **joint6 法兰盘中心** 的坐标，不是夹爪末端！

**Important**: `send_coords(x, y, z, rx, ry, rz)` controls the **joint6 flange center** coordinates, NOT the gripper tip!

- **法兰盘中心 (Flange center)**: joint6_flange link
- **夹爪基座偏移 (Gripper base offset)**: +34mm 沿 Z 轴
- **夹爪手指长度 (Gripper finger length)**: 约 40-50mm

如果需要精确控制夹爪末端位置，需要考虑总偏移约 **80-90mm**。

---

## 2. 坐标系定义 / Coordinate System Definition

### 机械臂基座坐标系 / Arm Base Frame

```
         Z ↑ (up)
           |
           |
    Y ←----O----→ X (forward)
          /
         / (left)
```

- **X轴 (X-axis)**: 机械臂前方 (forward)
- **Y轴 (Y-axis)**: 机械臂左侧 (left)
- **Z轴 (Z-axis)**: 向上 (up)

### 摄像头坐标系示例 / Camera Frame Example (RealSense D435)

```
    Z → (forward, depth direction)
    |
    |
    O----→ X (right)
    |
    ↓
    Y (down)
```

---

## 3. 如何标定转换参数 / How to Calibrate Transform Parameters

### 方法 1: 手动测量 / Method 1: Manual Measurement

1. **测量平移 (Measure translation)**:
   - 用卷尺测量摄像头原点到机械臂基座的距离
   - Measure distance from camera origin to arm base with a tape measure
   - 记录 X, Y, Z 偏移（单位：毫米）
   - Record X, Y, Z offset (in mm)

2. **测量旋转 (Measure rotation)**:
   - 使用量角器或倾角仪测量摄像头的安装角度
   - Use protractor or inclinometer to measure camera mounting angles
   - 记录 RX, RY, RZ 旋转角度（单位：度）
   - Record RX, RY, RZ rotation angles (in degrees)

### 方法 2: 使用标定板 / Method 2: Using Calibration Target

1. 在机械臂工作空间内放置已知位置的标定板
   Place a calibration target at a known position in the arm workspace

2. 使用摄像头检测标定板位置（摄像头坐标系）
   Use camera to detect calibration target position (in camera frame)

3. 用机械臂移动到同一位置（机械臂坐标系）
   Move arm to the same position (in arm frame)

4. 计算两个坐标系之间的变换关系
   Calculate the transformation between the two coordinate systems

### 方法 3: 在线微调 / Method 3: Online Fine-tuning

1. 启用坐标转换，初始值设为零
   Enable transform with initial values of zero

2. 发布一个测试目标坐标
   Publish a test target coordinate

3. 观察机械臂实际到达的位置与期望位置的差异
   Observe the difference between actual and desired position

4. 调整转换参数并重复测试
   Adjust transform parameters and repeat

---

## 4. 配置步骤 / Configuration Steps

### Step 1: 创建配置文件 / Create Config File

```bash
cd ~/mycobot_ws/src/my_cobot_control/config
cp camera_transform_example.yaml camera_transform.yaml
```

### Step 2: 编辑参数 / Edit Parameters

编辑 `camera_transform.yaml`，填入您测量的值：

Edit `camera_transform.yaml` with your measured values:

```yaml
/**:
  ros__parameters:
    use_camera_transform: true
    
    # 填入您的测量值 / Fill in your measured values
    camera_to_base_x: 150.0     # mm
    camera_to_base_y: -200.0    # mm
    camera_to_base_z: 300.0     # mm
    camera_to_base_rx: 0.0      # degrees
    camera_to_base_ry: 0.0      # degrees
    camera_to_base_rz: 0.0      # degrees
```

### Step 3: 在启动文件中加载配置 / Load Config in Launch File

修改您的启动文件：

Modify your launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('my_cobot_control'),
        'config',
        'camera_transform.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='my_cobot_control',
            executable='mycobot_controller',
            name='mycobot_controller',
            namespace='arm',
            parameters=[config_file],
            output='screen'
        ),
    ])
```

### Step 4: 测试转换 / Test Transform

```bash
# 启动控制节点
ros2 run my_cobot_control mycobot_controller --ros-args --params-file camera_transform.yaml

# 在另一个终端发布测试目标
ros2 topic pub --once /arm/target_pick geometry_msgs/msg/Point "{x: 100.0, y: 0.0, z: 50.0}"

# 观察日志输出，会显示：
# [PICK] Camera coords: (100.0, 0.0, 50.0) mm
# [PICK] Base coords:   (250.0, -200.0, 350.0) mm
```

---

## 5. 常见摄像头安装配置示例 / Common Camera Mount Examples

### A. 摄像头固定在小车上方 / Camera Fixed Above Workspace

```yaml
# 摄像头距离机械臂基座：前方 200mm，左侧 100mm，上方 500mm
# Camera offset: 200mm forward, 100mm left, 500mm up
# 摄像头朝下看（旋转180度）
# Camera looking down (rotated 180°)

camera_to_base_x: 200.0
camera_to_base_y: 100.0
camera_to_base_z: 500.0
camera_to_base_rx: 180.0    # 绕X轴旋转180° (flip upside down)
camera_to_base_ry: 0.0
camera_to_base_rz: 0.0
```

### B. 摄像头安装在小车前方 / Camera Mounted on Front of Rover

```yaml
# 摄像头在机械臂前方 250mm，高度 200mm
# Camera 250mm in front, 200mm high
# 向下倾斜 30 度
# Tilted down 30 degrees

camera_to_base_x: 250.0
camera_to_base_y: 0.0
camera_to_base_z: 200.0
camera_to_base_rx: 0.0
camera_to_base_ry: -30.0    # 向下倾斜 (tilted down)
camera_to_base_rz: 0.0
```

### C. 摄像头与机械臂基座对齐 / Camera Aligned with Arm Base

```yaml
# 摄像头与机械臂坐标系完全对齐（无需转换）
# Camera perfectly aligned with arm frame (no transform needed)

use_camera_transform: false
# 或者 / or:
camera_to_base_x: 0.0
camera_to_base_y: 0.0
camera_to_base_z: 0.0
camera_to_base_rx: 0.0
camera_to_base_ry: 0.0
camera_to_base_rz: 0.0
```

---

## 6. 验证转换正确性 / Verify Transform Correctness

### 测试方法 / Test Procedure:

1. **选择一个已知位置的物体**
   Choose an object at a known position

2. **用摄像头检测物体坐标**
   Detect object position with camera
   ```bash
   # 假设摄像头检测到: (x_cam, y_cam, z_cam) = (100, 50, 80)
   ```

3. **发布pick命令**
   Publish pick command
   ```bash
   ros2 topic pub --once /arm/target_pick geometry_msgs/msg/Point "{x: 100.0, y: 50.0, z: 80.0}"
   ```

4. **观察机械臂是否准确到达目标位置**
   Observe if arm reaches the target accurately

5. **如果偏差过大，调整转换参数**
   If error is large, adjust transform parameters

---

## 7. 高级：使用 TF2 进行转换 / Advanced: Using TF2 Transform

如果您的系统已经使用 ROS TF2 广播摄像头和机械臂之间的变换，可以修改代码直接使用 TF2：

If your system already uses ROS TF2 to broadcast transforms, you can modify the code to use TF2 directly:

```python
# 在 __init__ 中添加 TF2 buffer
self.tf_buffer = tf2_ros.Buffer()
self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

# 在 _transform_camera_to_base 中使用 TF2
def _transform_camera_to_base(self, x_cam, y_cam, z_cam):
    try:
        # 查找从 camera_link 到 base_link 的变换
        transform = self.tf_buffer.lookup_transform(
            'base_link', 'camera_link', rclpy.time.Time())
        
        # 应用变换
        point_cam = PointStamped()
        point_cam.header.frame_id = 'camera_link'
        point_cam.point.x = x_cam / 1000.0  # mm to m
        point_cam.point.y = y_cam / 1000.0
        point_cam.point.z = z_cam / 1000.0
        
        point_base = tf2_geometry_msgs.do_transform_point(point_cam, transform)
        
        return (point_base.point.x * 1000.0,  # m to mm
                point_base.point.y * 1000.0,
                point_base.point.z * 1000.0)
    except Exception as e:
        self.get_logger().error(f'TF lookup failed: {e}')
        return x_cam, y_cam, z_cam
```

---

## 8. 故障排查 / Troubleshooting

### 问题 1: 机械臂位置偏差很大 / Problem 1: Large Position Error

**原因 / Cause**: 转换参数不正确

**解决 / Solution**: 
- 重新测量摄像头和机械臂的相对位置
- 确认旋转角度的正负号
- 确认旋转顺序（当前代码使用 Z-Y-X 欧拉角）

### 问题 2: X 和 Y 轴互换 / Problem 2: X and Y Axes Swapped

**原因 / Cause**: 摄像头和机械臂的坐标系定义不同

**解决 / Solution**:
- 检查摄像头坐标系定义
- 可能需要交换 X 和 Y 的值
- 或者添加 90° 旋转

### 问题 3: 转换后坐标超出工作范围 / Problem 3: Transformed Coordinates Out of Range

**原因 / Cause**: 平移参数设置过大

**解决 / Solution**:
- 检查 camera_to_base_x/y/z 的值
- 确认单位是毫米（mm）而不是米（m）

---

## 参考资料 / References

- ROS 2 TF2 文档: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Tf2-Main.html
- 欧拉角与旋转矩阵: https://en.wikipedia.org/wiki/Euler_angles
- MyCobot 280 坐标系: 参见 CALIBRATION_GUIDE.md

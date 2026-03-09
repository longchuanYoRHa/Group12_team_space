# 摄像头坐标转换 - 快速开始 / Camera Transform - Quick Start

## 快速回答您的问题 / Quick Answers

### 1️⃣ `send_coords` 控制的是什么坐标？

**`send_coords(x, y, z, rx, ry, rz)` 控制的是 joint6 法兰盘中心的坐标，不是夹爪末端！**

- 📍 控制点：joint6_flange 中心
- 📏 夹爪基座偏移：+34mm (沿 Z 轴)
- ✋ 夹爪手指长度：约 40-50mm
- 📐 总偏移量：约 80-90mm

```
        ┌─────────────┐
        │ 夹爪手指尖端  │ ← 实际抓取点 (约 +80mm)
        └─────────────┘
             ││
        ┌─────────────┐
        │  夹爪基座   │ ← (+34mm)
        └─────────────┘
             ││
        ┌─────────────┐
        │ joint6法兰盘 │ ← send_coords 控制这里！
        └─────────────┘
```

### 2️⃣ 如何添加摄像头坐标转换？

#### 方法 A: 使用配置文件（推荐）

**步骤 1**: 创建配置文件
```bash
cd ~/mycobot_ws/src/my_cobot_control/config
nano camera_transform.yaml
```

**步骤 2**: 填入您测量的转换参数
```yaml
/**:
  ros__parameters:
    use_camera_transform: true
    
    # 摄像头相对于机械臂基座的位置 (单位: mm)
    camera_to_base_x: 200.0    # 前后方向
    camera_to_base_y: -100.0   # 左右方向
    camera_to_base_z: 300.0    # 上下方向
    
    # 摄像头相对于机械臂基座的旋转 (单位: 度)
    camera_to_base_rx: 0.0     # 绕X轴旋转
    camera_to_base_ry: -30.0   # 绕Y轴旋转 (例如: 向下倾斜30度)
    camera_to_base_rz: 0.0     # 绕Z轴旋转
```

**步骤 3**: 启动时加载配置
```bash
ros2 run my_cobot_control mycobot_controller \
  --ros-args --params-file ~/mycobot_ws/src/my_cobot_control/config/camera_transform.yaml
```

#### 方法 B: 直接在命令行指定参数

```bash
ros2 run my_cobot_control mycobot_controller \
  --ros-args \
  -p use_camera_transform:=true \
  -p camera_to_base_x:=200.0 \
  -p camera_to_base_y:=-100.0 \
  -p camera_to_base_z:=300.0 \
  -p camera_to_base_rx:=0.0 \
  -p camera_to_base_ry:=-30.0 \
  -p camera_to_base_rz:=0.0
```

---

## 测试转换是否正确 / Test Transform

### 1. 使用测试脚本

```bash
cd ~/mycobot_ws/src/my_cobot_control/scripts
python3 test_camera_transform.py
```

这个脚本会：
- ✅ 显示示例转换
- ✅ 提供交互式测试
- ✅ 生成 ROS 参数配置

### 2. 实际测试流程

```bash
# 终端 1: 启动控制器
ros2 run my_cobot_control mycobot_controller \
  --ros-args --params-file camera_transform.yaml

# 终端 2: 发送测试目标
ros2 topic pub --once /arm/target_pick geometry_msgs/msg/Point \
  "{x: 100.0, y: 50.0, z: 80.0}"

# 观察日志输出:
# [INFO] [PICK] Camera coords: (100.0, 50.0, 80.0) mm
# [INFO] [PICK] Base coords:   (300.0, -50.0, 380.0) mm
```

---

## 如何标定转换参数？ / How to Calibrate?

### 简单方法：手动测量

1. **测量摄像头位置**
   - 用卷尺测量摄像头到机械臂基座的距离
   - 记录 X, Y, Z 偏移（单位：毫米）

2. **测量摄像头角度**
   - 使用手机倾角仪或量角器
   - 记录绕 X, Y, Z 轴的旋转角度

3. **填入配置文件**
   ```yaml
   camera_to_base_x: 测量值  # mm
   camera_to_base_y: 测量值  # mm
   camera_to_base_z: 测量值  # mm
   camera_to_base_rx: 测量值 # degrees
   camera_to_base_ry: 测量值 # degrees
   camera_to_base_rz: 测量值 # degrees
   ```

### 精确方法：使用标定板

请参考详细指南：[CAMERA_TRANSFORM_GUIDE.md](../../CAMERA_TRANSFORM_GUIDE.md)

---

## 常见场景配置示例 / Common Scenarios

### 场景 1: 摄像头在机械臂上方

```yaml
# 摄像头安装在上方 500mm，朝下看
camera_to_base_x: 0.0
camera_to_base_y: 0.0
camera_to_base_z: 500.0
camera_to_base_rx: 180.0   # 翻转朝下
camera_to_base_ry: 0.0
camera_to_base_rz: 0.0
use_camera_transform: true
```

### 场景 2: 摄像头在小车前方

```yaml
# 摄像头在前方 250mm，高度 200mm，向下倾斜 30°
camera_to_base_x: 250.0
camera_to_base_y: 0.0
camera_to_base_z: 200.0
camera_to_base_rx: 0.0
camera_to_base_ry: -30.0   # 向下倾斜
camera_to_base_rz: 0.0
use_camera_transform: true
```

### 场景 3: 摄像头与机械臂对齐

```yaml
# 不需要转换
use_camera_transform: false
```

---

## 坐标系约定 / Coordinate System Convention

### 机械臂基座坐标系 (Base Frame)
```
    Z ↑ (向上 up)
      |
      |
Y ←---O---→ X (向前 forward)
     /
    / (向左 left)
```

### 摄像头坐标系 (Camera Frame - RealSense D435)
```
Z → (深度方向 depth, forward)
|
|
O---→ X (右 right)
|
↓
Y (下 down)
```

**转换顺序**: Z-Y-X 欧拉角 (Euler angles)

---

## 验证清单 / Verification Checklist

使用这个清单确保转换正确：

- [ ] 摄像头检测到的物体，机械臂能准确到达
- [ ] X 轴方向正确（前后）
- [ ] Y 轴方向正确（左右）
- [ ] Z 轴方向正确（上下）
- [ ] 位置误差在可接受范围内 (< 5mm)

---

## 调试技巧 / Debugging Tips

### 问题：机械臂位置偏差很大

**检查项**：
1. 确认单位是毫米 (mm)，不是米 (m)
2. 确认旋转角度是度 (degrees)，不是弧度 (radians)
3. 重新测量摄像头位置和角度
4. 检查坐标系定义是否一致

### 问题：X 和 Y 轴互换

**原因**: 摄像头和机械臂的坐标系定义不同

**解决**: 
- 交换 camera_to_base_x 和 camera_to_base_y 的值
- 或添加 90° 旋转

### 查看实时日志

```bash
ros2 run my_cobot_control mycobot_controller \
  --ros-args --params-file camera_transform.yaml \
  --log-level debug
```

---

## 文件列表 / File List

- ✅ [mycobot_controller.py](../my_cobot_control/mycobot_controller.py) - 主控制器（已添加转换功能）
- ✅ [camera_transform_example.yaml](../config/camera_transform_example.yaml) - 配置示例
- ✅ [test_camera_transform.py](test_camera_transform.py) - 测试脚本
- 📖 [CAMERA_TRANSFORM_GUIDE.md](../../CAMERA_TRANSFORM_GUIDE.md) - 详细指南

---

## 需要帮助？ / Need Help?

详细指南请查看: [CAMERA_TRANSFORM_GUIDE.md](../../CAMERA_TRANSFORM_GUIDE.md)

包含：
- 详细的坐标系说明
- 多种标定方法
- 高级用法 (TF2 集成)
- 故障排查指南

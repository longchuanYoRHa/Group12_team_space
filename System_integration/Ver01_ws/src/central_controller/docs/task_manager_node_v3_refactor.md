# `task_manager_node_v3.py` 拆分说明

## 目标

这次重构基于以下两部分已有工作继续推进：

- `central_controller/task_manager_node_v2.py`：完整但高度耦合的单文件实现
- `central_controller/task_manager_refactor_v2/`：已经验证过“主节点 + 子模块”的拆分方向

重构后的目标不是改写状态机行为，而是在尽量保持 ROS 接口、状态流转和外部依赖不变的前提下，把 `task_manager_node_v3.py` 变成一个“装配层”，把复杂逻辑下沉到可独立阅读和维护的模块中。

## 拆分后的文件结构

### 入口文件

- `central_controller/task_manager_node_v3.py`

职责：

- 创建 ROS2 节点
- 初始化共享运行时状态
- 初始化 publisher / subscription / action client / service
- 统一事件分发 `dispatch()`
- 驱动定时器节拍 `_state_timer_callback()`

### 模型层

- `central_controller/task_manager_v3_refactor/models.py`

职责：

- 定义 `TaskState`
- 定义 `CargoState`
- 定义 `NavPurpose`
- 定义 `TickEvent`、`ObjectVisionEvent`、`BinVisionEvent`、`ExploreFinishedEvent`
- 定义 Nav2 回调事件包装类型

这样做的好处是主节点、导航逻辑和视觉逻辑共享同一组状态/事件定义，不再把枚举和事件散落在主文件里。

### Docking 兼容层

- `central_controller/task_manager_v3_refactor/docking.py`

职责：

- 统一处理 `DockRobot` 在不同环境下的导入兼容
- 对 `nav2_msgs.action` 和 `opennav_docking_msgs.action` 做兜底

这样主节点和精确对位模块都不需要重复写导入兼容代码。

### 导航层

- `central_controller/task_manager_v3_refactor/navigation.py`

职责：

- TF 坐标变换
- `explore/resume` 控制发布
- Nav2 目标发送与取消
- Nav2 goal/result 回调处理
- Nav2 完成后进入 `PRECISION_ALIGN` 的桥接逻辑

这一层只关心“怎么导航”和“导航完成后怎么把结果交回状态机”，不处理抓取、放置和地图回退。

### 机械臂层

- `central_controller/task_manager_v3_refactor/arm.py`

职责：

- 订阅的机械臂状态更新
- 抓取结果处理
- 放置结果处理
- 发送抓取点到 `/arm/target_pick`
- 发送放置点到 `/arm/target_place`

这一层只关心机械臂动作与结果判定，不负责决定何时开始探索或去下一个兴趣点。

### 探索与视觉层

- `central_controller/task_manager_v3_refactor/exploration.py`

职责：

- INIT 阶段的就绪检测和 odom reset
- object / bin 视觉触发处理
- 探索结束后的地图保存
- PGM 兴趣点提取与过滤
- fallback 兴趣点导航
- 放置完成后的 explore 恢复逻辑

这是任务流中“感知驱动状态变化”的核心层。

### 精确对位层

- `central_controller/task_manager_v3_refactor/alignment.py`

职责：

- 进入 `PRECISION_ALIGN`
- 基于视觉生成 DockRobot 目标
- 处理 DockRobot 完成结果
- 动作完成后的后退恢复
- 超时跳过兴趣点

这部分从原主节点中单独拆出后，`PRECISION_ALIGN -> GRASP/PLACE/BACKUP` 的路径会更清晰。

## 当前运行链路

整体状态流仍保持原 `v3` 设计：

1. `INIT`
2. 可选 `PRE_EXPLORE_SPIN`
3. `EXPLORE`
4. 发现 object 后暂停探索并进入 `PRECISION_ALIGN`（物体预抓取对位）
5. 对位完成进入 `GRASP`
6. 抓取成功后后退并进入 `RESUME_EXPLORE_FOR_BIN`
7. 发现 bin 后进入 `NAV_TO_BIN_PREPLACE`
8. 再次 `PRECISION_ALIGN`
9. 对位完成进入 `PLACE_IN_BIN`
10. 放置成功后后退，再回到 `EXPLORE` 或 fallback 兴趣点导航

如果探索结束但仍有未完成目标，则会：

1. 保存地图
2. 从 PGM 中提取兴趣点
3. 过滤黑名单点
4. 进入 `NAV_TO_INTEREST_POINT`
5. 在兴趣点附近等待视觉触发 object 或 bin

## 主节点现在怎么工作

`task_manager_node_v3.py` 现在只负责三件事：

### 1. 维护共享状态

例如：

- `self.state`
- `self.cargo_state`
- `self.object_pose`
- `self.bin_pose`
- `self.cached_bin_poses`
- `self.object_blacklist`
- `self.bin_blacklist`

这些状态仍然集中挂在节点实例上，避免在多个模块之间来回复制上下文。

### 2. 装配 ROS 接口

包括：

- publisher
- subscription
- Nav2 action client
- DockRobot action client
- `/reset_odometry` client
- `task_manager/get_state` service

### 3. 统一事件分发

现在所有异步入口都会先归一化成事件，再交给 `dispatch()`：

- 定时器 -> `TickEvent`
- object 视觉 -> `ObjectVisionEvent`
- bin 视觉 -> `BinVisionEvent`
- explore 完成 -> `ExploreFinishedEvent`
- Nav2 goal/result -> 对应事件包装

这让主状态机入口固定在一个位置，后续继续做更细的拆分时不会破坏外部接口。

## 相比单文件版本的改进

- 主节点文件职责更单一，阅读入口更清楚
- 导航、机械臂、探索、对位的边界更明确
- `DockRobot` 兼容导入不再重复出现
- 事件模型集中定义，减少回调直接修改状态的散乱感
- 后续如果要继续把共享状态收敛到 dataclass/context，对现有结构影响会更小

## 兼容性说明

- 保留了原有 `main()` 入口
- 新增了 `setup.py` 中的 `task_manager_v3` console script
- 保留了 `TaskManagerNodeV2 = TaskManagerNodeV3` 的兼容别名，降低外部旧引用直接失效的风险

## 后续可继续优化的方向

- 把节点上的共享状态进一步收敛成 `RuntimeContext` dataclass
- 为 `dispatch()` 增加显式状态迁移日志
- 将 PGM fallback 和 CSV 记录再拆成独立服务类
- 给 `PRECISION_ALIGN` 和 fallback 流程补充更细粒度的测试

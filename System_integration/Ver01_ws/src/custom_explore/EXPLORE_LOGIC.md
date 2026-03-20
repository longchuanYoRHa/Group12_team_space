# custom_explore 探索逻辑说明（frontier / navgoal / 结束条件）

该节点 `custom_explore_node` 是对 `m-explore-ros2`（`explore_lite`）探索逻辑的复制，并做了一个关键改动：**默认处于等待状态**，只有当订阅到 `explore/resume` 为 `true` 后才开始真正下发 `nav2 NavigateToPose` 目标；当 `explore/resume=false` 时会停止探索并取消当前 nav2 目标。

节点接口：

- 订阅：`/explore/resume`（`std_msgs/Bool`）
- 发布：`/explore/finished`（`std_msgs/Bool`，`true` 表示探索结束）

---

## 1. frontier（边界点/前沿）如何判断

边界前沿使用 `frontier_exploration::FrontierSearch` 从代价地图（`nav2_costmap_2d::Costmap2D`）中计算，核心规则如下：

1. **起点**：以机器人在 costmap 全局坐标系中的位置为起点，先找到一个距离该起点附近的 `FREE_SPACE` 作为 BFS 初始起点。
2. **候选 frontier cell**（新前沿格子）的条件 `isNewFrontierCell(idx)`：
   - 该格子必须是 `NO_INFORMATION`（未知/未探索区域）
   - 且在它的 **4-邻域**（上/下/左/右）中至少存在一个 `FREE_SPACE`（自由空间）。
3. **frontier 聚类**：对一个初始候选格子，使用 **8-邻域 BFS** 把同一片前沿区域扩展为一个 frontier（`buildNewFrontier`），把所有满足候选条件的格子都加入该 frontier。
4. **frontier 大小过滤**：frontier 的大小阈值为：
   - `frontier.size * resolution >= min_frontier_size`  
   小于阈值的前沿会被丢弃。
5. **frontier centroid**：对前沿内所有格子的世界坐标取平均，得到该 frontier 的 `centroid`（后续导航目标使用它的坐标）。
6. **frontier cost 与排序**：每个 frontier 的 cost 为：
   - `cost = potential_scale * min_distance * resolution - gain_scale * size * resolution`
   其中 `min_distance` 是该前沿区域中到机器人最近格子的距离。
   最终把所有 frontier 按 `cost` **从小到大**排序（cost 越小越优先）。

因此，所谓“边界判断”最终落在：**未知格子 + 靠近已知自由空间 的连通区域**，并用 centroid 作为 nav 目标。

---

## 2. navgoal 如何依次下发（frontier 切换逻辑）

每次定时触发 `makePlan()` 时，会执行：

1. 读取当前机器人位姿：`pose = costmap_client_.getRobotPose()`
2. 计算所有 frontier：`frontiers = search_.searchFrom(pose.position)`
3. 如果开启可视化，会把 frontier marker 发布出来（并把“黑名单点”着色为红色）。
4. 选择下一目标：
   - 遍历排序后的 `frontiers`，找到**第一个不在黑名单（blacklist）中的 frontier**
   - 目标点 `target_position = frontier->centroid`
5. “同一个目标不重复下发”：
   - 若 `same_point(prev_goal_, target_position)` 返回 `true`（距离 < 0.2m），则本次不重复发 navgoal。
6. 否则发送 `nav2_msgs/action/NavigateToPose` goal：
   - `goal.pose.pose.position = target_position`
   - `goal.pose.header.frame_id = costmap_client_.getGlobalFrameID()`
   - 使用 `async_send_goal`，并设置 `result_callback` 到 `reachedGoal(...)`

导航结果回调 `reachedGoal` 的行为（SUCCEEDED/ABORTED/CANCELED）：

- `SUCCEEDED`：探索立即再次调用 `makePlan()`，选下一个 frontier
- `ABORTED`：把该 frontier 加入 blacklist，然后返回（等待下一次 timer tick 再重新选目标）
- `CANCELED`：直接返回（同样等待下一次 timer tick）

---

## 3. 什么时候结束探索（结束条件）

在 `makePlan()` 中，出现以下任意一种情况就会结束探索并发布：

- `frontiers.empty()`  
  说明当前 costmap/机器人位置下没有可用 frontier，调用 `stop(true)` 并发布 `/explore/finished=true`
- 所有 frontier 都被 blacklist 覆盖  
  也就是 `find_if_not(goalOnBlacklist)` 找不到非黑名单 frontier，调用 `stop(true)` 并发布 `/explore/finished=true`

`stop(true)` 同时会做两件事：

- `async_cancel_all_goals()`：取消 nav2 当前所有目标
- `exploring_timer_->cancel()`：停止定时规划

---

## 4. 什么时候取消一个“尝试”（cancel / blacklist 的含义）

这个节点里“取消一个尝试”主要分两层理解：

1. **外部强制停止一次尝试（explore/resume=false 或探索结束）**
   - 调用 `stop(false)`：取消所有 nav2 goals，并停止 timer
2. **内部判定该 frontier “没有进展”（进度超时）**
   - 如果在 `progress_timeout` 时间内没有检测到“到目标的最小距离（min_distance）持续下降”或目标点发生变化，
     当前 frontier centroid 会加入 blacklist
   - 之后会立刻递归调用 `makePlan()` 去选下一个 frontier，并下发新 nav goal
   - 注意：该逻辑**不会显式调用** `async_cancel_all_goals()`；它依赖 nav2 action 状态变化（例如 ABORTED）或新目标下发产生的行为来切换尝试
3. **nav2 action ABORTED**
   - `reachedGoal(ABORTED)` 会把该 frontier 加入 blacklist，但不会立即 `makePlan()`（会等 timer tick）

---

## 5. 可调参数（哪些建议改）

在节点上可通过 ROS2 参数调整：

- `planner_frequency`  
  `makePlan()` 的调用频率（等待/运行时都会由 timer 驱动）
- `progress_timeout`（默认 `30.0`，秒）  
  进度超时进入 blacklist 的阈值；仿真/地图更新慢时通常需要适当增大
- `visualize`（默认 `false`）  
  发布 frontier 可视化 marker（代价地图较大时会有额外开销）
- `potential_scale`（默认 `1e-3`）  
  frontier cost 的距离权重（影响排序）
- `orientation_scale`（默认 `0.0`）  
  与 `explore_lite` 保持一致：目前该节点里没有实际参与 frontier cost 计算（因此主要作为占位/兼容参数）
- `gain_scale`（默认 `1.0`）  
  frontier cost 的规模权重（影响排序）
- `min_frontier_size`（默认 `0.5`，单位等价为 `size * resolution`）  
  小前沿过滤阈值
- `return_to_init`（默认 `false`）  
  探索结束后导航回初始位姿（依赖 tf 获取初始 pose）
- `robot_base_frame`（默认 `base_link`）  
  用于 tf 获取机器人 pose / 初始位姿

此外，底层 costmap 订阅也有参数（来自 `Costmap2DClient`）：

- `costmap_topic`（默认 `costmap`）
- `costmap_updates_topic`（默认 `costmap_updates`）
- `transform_tolerance`（默认 `0.3`，秒）

---

## 6. 关键点：blacklist 命中规则

当判断某个候选 frontier 是否“在 blacklist”时，使用 centroid 的空间容差：

- `x_diff < 5 * resolution` 且 `y_diff < 5 * resolution` 认为同一个目标（命中）

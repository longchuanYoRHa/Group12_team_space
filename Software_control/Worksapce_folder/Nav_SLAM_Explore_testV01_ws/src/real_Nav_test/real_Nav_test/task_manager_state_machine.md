# Task Manager V2 状态机关系图

基于 `task_manager_node_v2.py` 中的 `TaskState` 与各回调/定时器逻辑整理。

## 状态说明

| 状态 | 含义 |
|------|------|
| **INIT** | 初始化，等待 Nav2 就绪并保存 home 位姿 |
| **EXPLORE** | 探索建图，可被物体/bin 检测打断 |
| **NAV_TO_OBJECT_PREGRASP** | 导航至物体预抓取位 |
| **GRASP** | 执行抓取（发 /arm/target_pick，由定时器根据 arm/gripper 状态判断结果） |
| **RESUME_EXPLORE_FOR_BIN** | 抓取成功后恢复探索以寻找 bin |
| **NAV_TO_BIN_PREPLACE** | 导航至 bin 预放置位 |
| **PLACE_IN_BIN** | 执行放置（发 /arm/target_place，由定时器根据 arm 状态判断结果） |
| **POST_ACTION** | 放置完成后、探索未结束时的收尾，随后回到 EXPLORE |
| **NAV_TO_INTEREST_POINT** | 探索结束后按兴趣点列表导航 |
| **WAIT_AT_INTEREST_POINT** | 在兴趣点等待视觉检测（物体或 bin）或超时 |

## Mermaid 状态图

```mermaid
stateDiagram-v2
    direction TB
    [*] --> INIT

    INIT --> EXPLORE : Nav2 就绪 & 保存 home 位姿

    EXPLORE --> NAV_TO_OBJECT_PREGRASP : 物体稳定检测(≥5 帧)\n暂停探索 & 发 Nav2 目标
    EXPLORE --> NAV_TO_INTEREST_POINT : 收到 explore/finished\n地图保存 & PGM 兴趣点有效

    NAV_TO_OBJECT_PREGRASP --> GRASP : Nav2 到达预抓取位

    GRASP --> RESUME_EXPLORE_FOR_BIN : 抓取成功\n(arm=holding, gripper=object_held)
    GRASP --> EXPLORE : 抓取失败达最大重试\n加入物体黑名单 & 恢复探索

    RESUME_EXPLORE_FOR_BIN --> NAV_TO_BIN_PREPLACE : bin 稳定检测(≥5 帧)\n暂停探索 & 发 Nav2 目标

    NAV_TO_BIN_PREPLACE --> PLACE_IN_BIN : Nav2 到达预放置位

    PLACE_IN_BIN --> NAV_TO_INTEREST_POINT : 放置成功 & explore_done_flag
    PLACE_IN_BIN --> POST_ACTION : 放置成功 & 探索未结束
    PLACE_IN_BIN --> RESUME_EXPLORE_FOR_BIN : 放置失败达最大重试

    POST_ACTION --> EXPLORE : 恢复探索

    NAV_TO_INTEREST_POINT --> WAIT_AT_INTEREST_POINT : Nav2 到达兴趣点\n开始等待计时

    WAIT_AT_INTEREST_POINT --> NAV_TO_OBJECT_PREGRASP : 物体稳定检测(空载)
    WAIT_AT_INTEREST_POINT --> NAV_TO_BIN_PREPLACE : bin 稳定检测(载物)
    WAIT_AT_INTEREST_POINT --> NAV_TO_INTEREST_POINT : 等待超时\n当前点加入黑名单 & 下一兴趣点
```

## 简化版（仅状态与主线）

```mermaid
stateDiagram-v2
    [*] --> INIT
    INIT --> EXPLORE
    EXPLORE --> NAV_TO_OBJECT_PREGRASP : 发现物体
    EXPLORE --> NAV_TO_INTEREST_POINT : 探索结束
    NAV_TO_OBJECT_PREGRASP --> GRASP : 到达
    GRASP --> RESUME_EXPLORE_FOR_BIN : 抓取成功
    GRASP --> EXPLORE : 抓取失败
    RESUME_EXPLORE_FOR_BIN --> NAV_TO_BIN_PREPLACE : 发现 bin
    NAV_TO_BIN_PREPLACE --> PLACE_IN_BIN : 到达
    PLACE_IN_BIN --> NAV_TO_INTEREST_POINT : 放置成功(已探索完)
    PLACE_IN_BIN --> POST_ACTION : 放置成功(未探索完)
    PLACE_IN_BIN --> RESUME_EXPLORE_FOR_BIN : 放置失败
    POST_ACTION --> EXPLORE
    NAV_TO_INTEREST_POINT --> WAIT_AT_INTEREST_POINT : 到达
    WAIT_AT_INTEREST_POINT --> NAV_TO_OBJECT_PREGRASP : 发现物体
    WAIT_AT_INTEREST_POINT --> NAV_TO_BIN_PREPLACE : 发现 bin
    WAIT_AT_INTEREST_POINT --> NAV_TO_INTEREST_POINT : 超时/下一兴趣点
```

## 驱动来源简要

- **INIT → EXPLORE**：定时器 `_state_timer_callback` 内 `_handle_init_state()`
- **EXPLORE → NAV_TO_OBJECT_PREGRASP**：视觉话题 `_object_point_callback`（/target_pick/*）
- **NAV_TO_OBJECT_PREGRASP → GRASP**：Nav2 `nav2_result_callback`（STATUS_SUCCEEDED + OBJECT_PREGRASP）
- **GRASP → RESUME_EXPLORE_FOR_BIN / EXPLORE**：定时器内 `_handle_grasp_arm_result()`（/arm/status, /arm/gripper_status）
- **RESUME_EXPLORE_FOR_BIN → NAV_TO_BIN_PREPLACE**：视觉话题 `_bin_point_callback`（/target_place/*）
- **NAV_TO_BIN_PREPLACE → PLACE_IN_BIN**：Nav2 `nav2_result_callback`（STATUS_SUCCEEDED + BIN_PREPLACE）
- **PLACE_IN_BIN → ***：定时器内 `_handle_place_arm_result()` 或 `_nav_to_next_interest_point()` / `_handle_post_action()`
- **EXPLORE → NAV_TO_INTEREST_POINT**：话题 `_explore_finished_callback`（explore/finished）+ 地图保存与兴趣点检测
- **NAV_TO_INTEREST_POINT → WAIT_AT_INTEREST_POINT**：Nav2 `nav2_result_callback`（INTEREST_POINT）
- **WAIT_AT_INTEREST_POINT → ***：`_object_point_callback` / `_bin_point_callback` 或定时器 `_handle_wait_at_interest_point_timeout()`

```mermaid
sequenceDiagram
    autonumber
    participant V as Vision(/target_pick/*,/target_place/*)
    participant TM as TaskManagerV2
    participant EX as ExploreNode(explore/resume, explore/finished)
    participant TF as TF2
    participant N2 as Nav2(navigate_to_pose)
    participant ARM as Manipulator(/arm/*)

    Note over TM: INIT
    TM->>N2: wait_for_server()
    TM->>TF: lookup(map<-base_link)
    TM->>EX: publish explore/resume=True
    Note over TM: EXPLORE

    Note over V,TM: 发现物体（空载，稳定≥5帧）
    V-->>TM: /target_pick/<color> (Point)
    TM->>TF: lookup(map<-camera) & transform
    TM->>EX: publish explore/resume=False
    TM->>N2: send_goal(NavPurpose=OBJECT_PREGRASP)
    Note over TM: NAV_TO_OBJECT_PREGRASP

    N2-->>TM: result STATUS_SUCCEEDED
    Note over TM: GRASP

    Note over V,TM: 抓取触发（GRASP状态下用camera->base_link）
    V-->>TM: /target_pick/<color> (Point)
    TM->>TF: lookup(base_link<-camera) & transform(mm)
    TM->>ARM: publish /arm/target_pick

    ARM-->>TM: /arm/status=holding
    ARM-->>TM: /arm/gripper_status=object_held
    Note over TM: RESUME_EXPLORE_FOR_BIN
    TM->>EX: publish explore/resume=True

    Note over V,TM: 发现bin（载物，稳定≥5帧）
    V-->>TM: /target_place/<color> (Point)
    TM->>TF: lookup(map<-camera) & transform
    TM->>EX: publish explore/resume=False
    TM->>N2: send_goal(NavPurpose=BIN_PREPLACE)
    Note over TM: NAV_TO_BIN_PREPLACE

    N2-->>TM: result STATUS_SUCCEEDED
    Note over TM: PLACE_IN_BIN

    Note over V,TM: 放置触发（PLACE状态下用Pose->base_link）
    V-->>TM: /target_place/<color> (Point)
    TM->>TF: lookup(base_link<-map) & transform(mm)
    TM->>ARM: publish /arm/target_place

    ARM-->>TM: /arm/status=idle
    alt explore_done_flag == false
        Note over TM: POST_ACTION -> EXPLORE
        TM->>EX: publish explore/resume=True
    else explore_done_flag == true
        Note over TM: NAV_TO_INTEREST_POINT
        TM->>N2: send_goal(NavPurpose=INTEREST_POINT)
        N2-->>TM: result STATUS_SUCCEEDED
        Note over TM: WAIT_AT_INTEREST_POINT
    end
```
#!/usr/bin/env python3
"""
任务管理器状态机节点
实现 Explore-Pick-Stow-SearchBin-Place 完整工作流程

该节点是系统的核心调度器，负责：
1. 控制探索行为（explore_lite）的启动和停止
2. 监听物体检测器（object_detector）和回收箱检测器（bin_detector）
3. 协调 Nav2 导航系统执行目标导航
4. 调用机械臂操作动作（抓取、存放、放置）
5. 管理状态转换和错误恢复机制
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
import tf2_ros
import geometry_msgs.msg as geometry_msgs
import nav2_msgs.action as nav2_msgs
import std_msgs.msg as std_msgs
from enum import Enum
import math
import time


class CargoState(Enum):
    """
    载物状态枚举
    用于跟踪机器人当前是否携带物体
    """
    EMPTY = "empty"          # 空载状态：机器人未携带物体，可以抓取新物体
    HAS_OBJECT = "has_object"  # 载物状态：机器人已携带物体，需要寻找回收箱


class TaskState(Enum):
    """
    任务状态枚举
    定义了完整工作流程中的所有状态
    """
    INIT = "init"                          # 初始化状态：等待系统就绪
    EXPLORE = "explore"                    # 探索状态：启动探索，监听物体检测
    OBJECT_FOUND = "object_found"          # 物体发现状态：检测到可回收物体
    PAUSE_EXPLORE = "pause_explore"        # 暂停探索状态：停止探索，准备导航
    NAV_TO_OBJECT_PREGRASP = "nav_to_object_pregrasp"  # 导航到物体预抓取位置
    PRECISION_ALIGN_OBJECT = "precision_align_object"  # 精确对准物体（可选）
    GRASP = "grasp"                        # 抓取状态：执行抓取动作
    STOW_ON_ROBOT = "stow_on_robot"        # 存放状态：将物体移动到车载存放位
    RESUME_EXPLORE_FOR_BIN = "resume_explore_for_bin"  # 恢复探索寻找回收箱
    BIN_FOUND = "bin_found"                # 回收箱发现状态：检测到回收箱
    NAV_TO_BIN_PREPLACE = "nav_to_bin_preplace"  # 导航到回收箱预放置位置
    PRECISION_ALIGN_BIN = "precision_align_bin"  # 精确对准回收箱（可选）
    PLACE_IN_BIN = "place_in_bin"          # 放置状态：将物体放入回收箱
    POST_ACTION = "post_action"            # 后处理状态：任务完成后的处理


class TaskManagerNode(Node):
    """
    任务管理器状态机节点
    
    这是系统的核心控制节点，实现了一个完整的状态机来协调：
    - 探索行为（explore_lite）
    - 导航系统（Nav2）
    - 物体检测（object_detector）
    - 回收箱检测（bin_detector）
    - 机械臂操作（grasp/stow/place actions）
    """
    
    def __init__(self):
        super().__init__('task_manager')
        
        # ========== 状态变量 ==========
        self.current_state = TaskState.INIT  # 当前任务状态
        self.cargo_state = CargoState.EMPTY  # 载物状态（空载/载物）
        self.home_pose = None  # 起始位置，用于任务完成后返回
        self.object_pose = None  # 检测到的物体位姿（map坐标系）
        self.bin_pose = None  # 检测到的回收箱位姿（map坐标系）
        self.stow_pose = None  # 车载存放位姿（arm_base坐标系，固定位置）
        
        # ========== 检测稳定性计数器 ==========
        # 用于多帧确认，避免误检测
        self.object_detection_count = 0  # 物体检测连续帧数
        self.bin_detection_count = 0  # 回收箱检测连续帧数
        self.required_detection_frames = 5  # 稳定检测所需连续帧数（N帧确认）
        
        # ========== 重试计数器 ==========
        # 用于失败恢复机制
        self.grasp_retry_count = 0  # 抓取重试次数
        self.max_grasp_retries = 2  # 最大抓取重试次数
        self.stow_retry_count = 0  # 存放重试次数
        self.max_stow_retries = 2  # 最大存放重试次数
        self.place_retry_count = 0  # 放置重试次数
        self.max_place_retries = 2  # 最大放置重试次数
        
        # ========== 失败物体黑名单 ==========
        # 记录抓取失败的物体位置，避免反复尝试
        self.object_blacklist = []  # 黑名单位置列表
        self.blacklist_radius = 0.3  # 黑名单半径（米），在此范围内的物体将被忽略
        
        # ========== Action 客户端 ==========
        # Nav2 导航动作客户端
        self.nav2_client = ActionClient(self, nav2_msgs.NavigateToPose, 'navigate_to_pose')
        self.nav2_goal_handle = None  # 当前导航目标句柄
        
        # ========== 发布器 ==========
        # 控制 explore_lite 的启动/停止（通过 /explore/resume topic）
        self.explore_control_pub = self.create_publisher(
            std_msgs.Bool, 'explore/resume', 10
        )
        # 发布当前状态（用于监控和调试）
        self.state_pub = self.create_publisher(
            std_msgs.String, 'task_manager/state', 10
        )
        # 发布载物状态（用于监控和调试）
        self.cargo_state_pub = self.create_publisher(
            std_msgs.String, 'task_manager/cargo_state', 10
        )
        
        # ========== 订阅器 ==========
        # TODO: 替换为实际的 object_detector topic
        # 订阅物体检测器发布的物体位姿
        self.object_pose_sub = self.create_subscription(
            geometry_msgs.PoseStamped,
            'object_detector/object_pose',  # 伪代码占位符
            self.object_pose_callback,
            qos_profile_sensor_data
        )
        
        # TODO: 替换为实际的 bin_detector topic
        # 订阅回收箱检测器发布的回收箱位姿
        self.bin_pose_sub = self.create_subscription(
            geometry_msgs.PoseStamped,
            'bin_detector/bin_pose',  # 伪代码占位符
            self.bin_pose_callback,
            qos_profile_sensor_data
        )
        
        # ========== TF 变换 ==========
        # 用于坐标系转换（map <-> base_link <-> arm_base 等）
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ========== 状态机定时器 ==========
        # 每 0.1 秒执行一次状态机逻辑（10Hz）
        self.state_timer = self.create_timer(0.1, self.state_machine_callback)
        
        # ========== 节点参数 ==========
        self.declare_parameter('pregrasp_distance', 0.5)  # 预抓取距离（米）
        self.declare_parameter('preplace_distance', 0.6)  # 预放置距离（米）
        self.declare_parameter('stow_pose_x', 0.3)  # 存放位姿 X 坐标
        self.declare_parameter('stow_pose_y', 0.0)  # 存放位姿 Y 坐标
        self.declare_parameter('stow_pose_z', 0.2)  # 存放位姿 Z 坐标
        
        self.get_logger().info('任务管理器节点已初始化')
        
    def object_pose_callback(self, msg):
        """
        物体位姿检测回调函数
        
        当物体检测器发布新的物体位姿时调用。
        仅在空载状态且处于探索状态时处理物体检测。
        使用多帧确认机制避免误检测。
        
        Args:
            msg: geometry_msgs.msg.PoseStamped，物体位姿（map坐标系）
        """
        # 只在空载且探索状态下处理物体检测
        if self.cargo_state == CargoState.EMPTY and self.current_state == TaskState.EXPLORE:
            # 检查物体是否在黑名单中（之前抓取失败的位置）
            if self.is_pose_in_blacklist(msg.pose.position):
                return  # 忽略黑名单中的物体
            
            # 更新物体位姿并增加检测计数
            self.object_pose = msg
            self.object_detection_count += 1
            
            # 达到稳定检测帧数，确认物体发现
            if self.object_detection_count >= self.required_detection_frames:
                self.get_logger().info('物体发现并确认！')
                self.current_state = TaskState.OBJECT_FOUND
                self.object_detection_count = 0
        else:
            # 不在正确状态，重置计数器
            self.object_detection_count = 0
    
    def bin_pose_callback(self, msg):
        """
        回收箱位姿检测回调函数
        
        当回收箱检测器发布新的回收箱位姿时调用。
        仅在载物状态且处于寻找回收箱的探索状态时处理。
        使用多帧确认机制避免误检测。
        
        Args:
            msg: geometry_msgs.msg.PoseStamped，回收箱位姿（map坐标系）
        """
        # 只在载物且寻找回收箱状态下处理回收箱检测
        if self.cargo_state == CargoState.HAS_OBJECT and self.current_state == TaskState.RESUME_EXPLORE_FOR_BIN:
            # 更新回收箱位姿并增加检测计数
            self.bin_pose = msg
            self.bin_detection_count += 1
            
            # 达到稳定检测帧数，确认回收箱发现
            if self.bin_detection_count >= self.required_detection_frames:
                self.get_logger().info('回收箱发现并确认！')
                self.current_state = TaskState.BIN_FOUND
                self.bin_detection_count = 0
        else:
            # 不在正确状态，重置计数器
            self.bin_detection_count = 0
    
    def is_pose_in_blacklist(self, position):
        """
        检查位置是否在黑名单中
        
        用于避免反复尝试抓取失败的物体。
        
        Args:
            position: geometry_msgs.msg.Point，要检查的位置
        
        Returns:
            bool: 如果位置在黑名单半径内返回 True，否则返回 False
        """
        for blacklist_pos in self.object_blacklist:
            # 计算欧氏距离
            distance = math.sqrt(
                (position.x - blacklist_pos.x)**2 +
                (position.y - blacklist_pos.y)**2
            )
            # 如果在黑名单半径内，返回 True
            if distance < self.blacklist_radius:
                return True
        return False
    
    def state_machine_callback(self):
        """
        状态机主回调函数
        
        这是状态机的核心执行函数，每 0.1 秒被调用一次。
        负责：
        1. 发布当前状态和载物状态（用于监控）
        2. 根据当前状态调用相应的处理函数
        3. 执行状态转换逻辑
        """
        # ========== 发布状态信息（用于监控和调试）==========
        state_msg = std_msgs.String()
        state_msg.data = self.current_state.value
        self.state_pub.publish(state_msg)
        
        cargo_msg = std_msgs.String()
        cargo_msg.data = self.cargo_state.value
        self.cargo_state_pub.publish(cargo_msg)
        
        # ========== 状态机逻辑分发 ==========
        # 根据当前状态调用相应的处理函数
        if self.current_state == TaskState.INIT:
            self.handle_init_state()
        elif self.current_state == TaskState.EXPLORE:
            self.handle_explore_state()
        elif self.current_state == TaskState.OBJECT_FOUND:
            self.handle_object_found_state()
        elif self.current_state == TaskState.PAUSE_EXPLORE:
            self.handle_pause_explore_state()
        elif self.current_state == TaskState.NAV_TO_OBJECT_PREGRASP:
            self.handle_nav_to_object_pregrasp_state()
        elif self.current_state == TaskState.PRECISION_ALIGN_OBJECT:
            self.handle_precision_align_object_state()
        elif self.current_state == TaskState.GRASP:
            self.handle_grasp_state()
        elif self.current_state == TaskState.STOW_ON_ROBOT:
            self.handle_stow_on_robot_state()
        elif self.current_state == TaskState.RESUME_EXPLORE_FOR_BIN:
            self.handle_resume_explore_for_bin_state()
        elif self.current_state == TaskState.BIN_FOUND:
            self.handle_bin_found_state()
        elif self.current_state == TaskState.NAV_TO_BIN_PREPLACE:
            self.handle_nav_to_bin_preplace_state()
        elif self.current_state == TaskState.PRECISION_ALIGN_BIN:
            self.handle_precision_align_bin_state()
        elif self.current_state == TaskState.PLACE_IN_BIN:
            self.handle_place_in_bin_state()
        elif self.current_state == TaskState.POST_ACTION:
            self.handle_post_action_state()
    
    def handle_init_state(self):
        """
        处理初始化状态
        
        等待系统就绪（TF变换、SLAM、Nav2），然后保存起始位置并进入探索状态。
        """
        # 等待 Nav2 服务器就绪
        if not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('等待 Nav2 服务器...')
            return
        
        # 保存起始位置（home_pose）
        try:
            # 获取当前机器人在 map 坐标系中的位姿
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            self.home_pose = geometry_msgs.PoseStamped()
            self.home_pose.header.frame_id = 'map'
            self.home_pose.pose.position.x = transform.transform.translation.x
            self.home_pose.pose.position.y = transform.transform.translation.y
            self.home_pose.pose.orientation = transform.transform.rotation
            
            self.get_logger().info('系统就绪，起始位置已保存')
            # 进入探索状态
            self.current_state = TaskState.EXPLORE
        except Exception as e:
            # TF 变换尚未就绪，继续等待
            self.get_logger().warn(f'等待 TF 变换: {e}')
    
    def handle_explore_state(self):
        """
        处理探索状态
        
        启动或恢复 explore_lite 探索行为。
        物体检测由 object_pose_callback 异步处理。
        """
        # 启动/恢复 explore_lite（通过发布 True 到 /explore/resume topic）
        explore_msg = std_msgs.Bool()
        explore_msg.data = True
        self.explore_control_pub.publish(explore_msg)
        
        # 注意：物体检测在 object_pose_callback 中异步处理
        # 状态转换（OBJECT_FOUND）也在回调中触发
    
    def handle_object_found_state(self):
        """
        处理物体发现状态
        
        当物体被稳定检测到时，立即进入暂停探索状态。
        """
        self.get_logger().info('物体发现，转换到暂停探索状态')
        self.current_state = TaskState.PAUSE_EXPLORE
    
    def handle_pause_explore_state(self):
        """
        处理暂停探索状态
        
        取消 Nav2 当前导航目标，停止 explore_lite，准备执行物体抓取任务。
        此状态用于物体抓取和回收箱放置两个场景。
        """
        # 取消 Nav2 当前导航目标
        if self.nav2_goal_handle is not None:
            self.nav2_client.cancel_goal_async(self.nav2_goal_handle)
            self.nav2_goal_handle = None
        
        # 停止 explore_lite（通过发布 False 到 /explore/resume topic）
        explore_msg = std_msgs.Bool()
        explore_msg.data = False
        self.explore_control_pub.publish(explore_msg)
        
        # 根据载物状态决定下一步
        # 如果空载，导航到物体；如果载物，导航到回收箱
        if self.cargo_state == CargoState.EMPTY:
            self.current_state = TaskState.NAV_TO_OBJECT_PREGRASP
        else:
            self.current_state = TaskState.NAV_TO_BIN_PREPLACE
    
    def handle_nav_to_object_pregrasp_state(self):
        """
        处理导航到物体预抓取位置状态
        
        计算物体前方的预抓取位置（保持安全距离，面向物体），
        然后使用 Nav2 导航到该位置。
        """
        # 检查物体位姿是否可用
        if self.object_pose is None:
            self.get_logger().error('物体位姿不可用！')
            self.current_state = TaskState.EXPLORE
            return
        
        # 获取预抓取距离参数
        pregrasp_distance = self.get_parameter('pregrasp_distance').value
        
        # 计算预抓取位置（物体前方，面向物体）
        goal_pose = self.calculate_pregrasp_pose(self.object_pose, pregrasp_distance)
        
        # 创建 Nav2 导航目标
        goal_msg = nav2_msgs.NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # 如果还没有发送目标，发送导航目标
        if self.nav2_goal_handle is None:
            self.get_logger().info('发送 Nav2 目标到物体预抓取位置')
            send_goal_future = self.nav2_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.nav2_goal_response_callback)
        else:
            # 检查导航目标状态
            # 状态值：1=ACCEPTED, 2=EXECUTING, 3=CANCELING, 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
            from rclpy.action import GoalStatus
            status = self.nav2_goal_handle.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                # 导航成功，到达预抓取位置
                self.get_logger().info('已到达预抓取位置')
                self.nav2_goal_handle = None
                self.current_state = TaskState.PRECISION_ALIGN_OBJECT
            elif status in [GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED]:
                # 导航失败，返回探索状态
                self.get_logger().warn('导航失败，返回探索状态')
                self.nav2_goal_handle = None
                self.current_state = TaskState.EXPLORE
    
    def handle_precision_align_object_state(self):
        """
        处理精确对准物体状态（可选但推荐）
        
        使用 D435i 相机进行厘米级精确对准，确保机械臂可达且误差可控。
        当前为伪代码实现，直接跳转到抓取状态。
        """
        # TODO: 实现使用 D435i 的精确对准逻辑
        # 当前为伪代码，直接跳转到抓取
        self.get_logger().info('精确对准（伪代码 - 跳转到抓取）')
        self.current_state = TaskState.GRASP
    
    def handle_grasp_state(self):
        """
        处理抓取状态
        
        调用机械臂抓取动作（grasp_server），抓取物体。
        包含重试机制和失败处理（添加到黑名单）。
        """
        # TODO: 调用实际的抓取动作（伪代码）
        self.get_logger().info('调用抓取动作（伪代码）')
        
        # 伪代码：调用 grasp_server
        # grasp_success = self.call_grasp_action(self.object_pose)
        grasp_success = True  # 占位符
        
        if grasp_success:
            # 抓取成功
            self.get_logger().info('抓取成功！')
            self.cargo_state = CargoState.HAS_OBJECT  # 更新载物状态
            self.grasp_retry_count = 0
            # 进入存放状态
            self.current_state = TaskState.STOW_ON_ROBOT
        else:
            # 抓取失败，重试
            self.grasp_retry_count += 1
            if self.grasp_retry_count >= self.max_grasp_retries:
                # 超过最大重试次数，放弃该物体
                self.get_logger().warn('抓取失败，超过重试次数，放弃该物体')
                # 添加到黑名单，避免反复尝试
                if self.object_pose:
                    self.object_blacklist.append(self.object_pose.pose.position)
                self.grasp_retry_count = 0
                # 返回探索状态
                self.current_state = TaskState.EXPLORE
            else:
                # 重试精确对准
                self.get_logger().info(f'抓取失败，重试中 ({self.grasp_retry_count}/{self.max_grasp_retries})')
                self.current_state = TaskState.PRECISION_ALIGN_OBJECT
    
    def handle_stow_on_robot_state(self):
        """
        处理车载存放状态（关键新增状态）
        
        将抓取的物体移动到车载存放位（stow pose）。
        成功后启用携带模式（调整 Nav2 参数），然后进入寻找回收箱的探索状态。
        """
        # TODO: 调用实际的存放动作（伪代码）
        self.get_logger().info('调用存放动作（伪代码）')
        
        # 伪代码：调用 stow_server
        # stow_success = self.call_stow_action()
        stow_success = True  # 占位符
        
        if stow_success:
            # 存放成功
            self.get_logger().info('存放成功！')
            self.stow_retry_count = 0
            # 启用携带模式（调整 Nav2 参数：降低速度，增大膨胀半径）
            self.adjust_nav2_for_carry_mode(True)
            # 进入寻找回收箱的探索状态
            self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
        else:
            # 存放失败，重试
            self.stow_retry_count += 1
            if self.stow_retry_count >= self.max_stow_retries:
                # 超过最大重试次数
                self.get_logger().warn('存放失败，超过重试次数')
                # 选项：放回桌面或继续夹持
                self.stow_retry_count = 0
                # 当前策略：继续夹持并继续执行
                self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
            else:
                # 重试抓取
                self.get_logger().info(f'存放失败，重试中 ({self.stow_retry_count}/{self.max_stow_retries})')
                self.current_state = TaskState.GRASP
    
    def handle_resume_explore_for_bin_state(self):
        """
        处理恢复探索寻找回收箱状态
        
        恢复 explore_lite 探索行为，将"回收箱"作为关键目标。
        回收箱检测由 bin_pose_callback 异步处理。
        """
        # 恢复 explore_lite（通过发布 True 到 /explore/resume topic）
        explore_msg = std_msgs.Bool()
        explore_msg.data = True
        self.explore_control_pub.publish(explore_msg)
        
        # 注意：回收箱检测在 bin_pose_callback 中异步处理
        # 状态转换（BIN_FOUND）也在回调中触发
    
    def handle_bin_found_state(self):
        """
        处理回收箱发现状态
        
        当回收箱被稳定检测到时，立即进入暂停探索状态。
        """
        self.get_logger().info('回收箱发现，转换到暂停探索状态')
        self.current_state = TaskState.PAUSE_EXPLORE
    
    def handle_nav_to_bin_preplace_state(self):
        """
        处理导航到回收箱预放置位置状态
        
        计算回收箱前方的预放置位置（保持安全距离，面向回收箱），
        然后使用 Nav2 导航到该位置。
        """
        # 检查回收箱位姿是否可用
        if self.bin_pose is None:
            self.get_logger().error('回收箱位姿不可用！')
            self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
            return
        
        # 获取预放置距离参数
        preplace_distance = self.get_parameter('preplace_distance').value
        
        # 计算预放置位置（回收箱前方，面向回收箱）
        goal_pose = self.calculate_pregrasp_pose(self.bin_pose, preplace_distance)
        
        # 创建 Nav2 导航目标
        goal_msg = nav2_msgs.NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # 如果还没有发送目标，发送导航目标
        if self.nav2_goal_handle is None:
            self.get_logger().info('发送 Nav2 目标到回收箱预放置位置')
            send_goal_future = self.nav2_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.nav2_goal_response_callback)
        else:
            # 检查导航目标状态
            from rclpy.action import GoalStatus
            status = self.nav2_goal_handle.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                # 导航成功，到达预放置位置
                self.get_logger().info('已到达预放置位置')
                self.nav2_goal_handle = None
                self.current_state = TaskState.PRECISION_ALIGN_BIN
            elif status in [GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED]:
                # 导航失败，返回寻找回收箱状态
                self.get_logger().warn('导航失败，返回寻找回收箱状态')
                self.nav2_goal_handle = None
                self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
    
    def handle_precision_align_bin_state(self):
        """
        处理精确对准回收箱状态（推荐）
        
        使用 D435i 相机进行精确对准，确保回收箱在相机视野中心、距离合适。
        当前为伪代码实现，直接跳转到放置状态。
        """
        # TODO: 实现使用 D435i 的精确对准逻辑
        # 当前为伪代码，直接跳转到放置
        self.get_logger().info('精确对准回收箱（伪代码 - 跳转到放置）')
        self.current_state = TaskState.PLACE_IN_BIN
    
    def handle_place_in_bin_state(self):
        """
        处理放置到回收箱状态
        
        执行放置动作（从存放位取出物体 → 放入回收箱）。
        包含重试机制和失败处理。
        """
        # TODO: 调用实际的放置动作（伪代码）
        self.get_logger().info('调用放置动作（伪代码）')
        
        # 伪代码：调用 place_server
        # place_success = self.call_place_action(self.bin_pose)
        place_success = True  # 占位符
        
        if place_success:
            # 放置成功
            self.get_logger().info('放置成功！')
            self.cargo_state = CargoState.EMPTY  # 更新载物状态为空载
            self.place_retry_count = 0
            # 恢复 Nav2 参数（禁用携带模式）
            self.adjust_nav2_for_carry_mode(False)
            # 进入后处理状态
            self.current_state = TaskState.POST_ACTION
        else:
            # 放置失败，重试
            self.place_retry_count += 1
            if self.place_retry_count >= self.max_place_retries:
                # 超过最大重试次数，恢复探索
                self.get_logger().warn('放置失败，超过重试次数，恢复探索')
                self.place_retry_count = 0
                self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
            else:
                # 重试精确对准
                self.get_logger().info(f'放置失败，重试中 ({self.place_retry_count}/{self.max_place_retries})')
                self.current_state = TaskState.PRECISION_ALIGN_BIN
    
    def handle_post_action_state(self):
        """
        处理后处理状态
        
        任务完成后的处理选项：
        - 返回探索状态继续寻找下一个物体
        - 返回起始位置（home）
        - 结束任务
        当前实现：返回探索状态。
        """
        # 选项：返回探索或返回起始位置
        self.get_logger().info('后处理：返回探索状态')
        self.current_state = TaskState.EXPLORE
    
    def calculate_pregrasp_pose(self, target_pose, distance):
        """
        计算预抓取/预放置位姿
        
        在目标前方计算一个位置，保持指定距离，面向目标。
        用于物体抓取和回收箱放置的导航目标计算。
        
        Args:
            target_pose: geometry_msgs.msg.PoseStamped，目标位姿（map坐标系）
            distance: float，保持的距离（米）
        
        Returns:
            geometry_msgs.msg.PoseStamped: 预抓取/预放置位姿（map坐标系）
        """
        goal_pose = geometry_msgs.PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # 计算从目标指向机器人的方向向量
        # 获取当前机器人位姿
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except:
            # TF 变换失败，使用默认值
            robot_x = 0.0
            robot_y = 0.0
        
        # 计算从目标指向机器人的向量
        dx = robot_x - target_pose.pose.position.x
        dy = robot_y - target_pose.pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)
        
        # 归一化方向向量
        if dist > 0:
            dx /= dist
            dy /= dist
        else:
            # 如果距离为0，使用默认方向
            dx = 1.0
            dy = 0.0
        
        # 目标位置：目标位置 + 距离 * 方向（朝向机器人）
        goal_pose.pose.position.x = target_pose.pose.position.x + distance * dx
        goal_pose.pose.position.y = target_pose.pose.position.y + distance * dy
        goal_pose.pose.position.z = 0.0
        
        # 朝向：面向目标
        yaw = math.atan2(
            target_pose.pose.position.y - goal_pose.pose.position.y,
            target_pose.pose.position.x - goal_pose.pose.position.x
        )
        
        # 将 yaw 角转换为四元数（仅绕 Z 轴旋转）
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return goal_pose
    
    def nav2_goal_response_callback(self, future):
        """
        Nav2 目标响应回调函数
        
        当 Nav2 接受或拒绝导航目标时调用。
        
        Args:
            future: 包含目标句柄的 Future 对象
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            # 目标被拒绝
            self.get_logger().error('Nav2 目标被拒绝！')
            self.nav2_goal_handle = None
            return
        
        # 目标被接受
        self.get_logger().info('Nav2 目标已接受')
        self.nav2_goal_handle = goal_handle
        
        # 获取结果回调（用于异步获取导航结果）
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_result_callback)
    
    def nav2_result_callback(self, future):
        """
        Nav2 目标结果回调函数
        
        当 Nav2 导航完成（成功/失败/取消）时调用。
        主要用于日志记录，实际状态检查在状态处理函数中进行。
        
        Args:
            future: 包含目标句柄的 Future 对象
        """
        from rclpy.action import GoalStatus
        goal_handle = future.result()
        if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 目标成功完成')
        elif goal_handle.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Nav2 目标被中止')
        elif goal_handle.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Nav2 目标被取消')
        # 注意：状态检查也在状态处理函数中进行，用于状态转换
    
    def adjust_nav2_for_carry_mode(self, enable):
        """
        调整 Nav2 参数以适应携带模式
        
        当机器人携带物体时，需要调整导航参数以提高安全性：
        - 降低最大线速度和角速度
        - 增大膨胀半径（inflation radius）
        - 如果物体伸出底盘外，增大 footprint
        
        Args:
            enable: bool，True 启用携带模式，False 恢复正常模式
        """
        # TODO: 实现 Nav2 参数调整
        # 可以通过以下方式实现：
        # 1. 动态重配置（Dynamic reconfigure）
        # 2. Nav2 服务调用
        # 3. 切换 costmap 参数文件
        if enable:
            self.get_logger().info('启用携带模式：降低速度，增大膨胀半径')
            # 伪代码：
            # nav2_params.max_vel_x = 0.3  # 从默认值降低
            # nav2_params.inflation_radius = 0.5  # 增大
        else:
            self.get_logger().info('禁用携带模式：恢复正常参数')
            # 伪代码：
            # nav2_params.max_vel_x = 0.5  # 恢复默认值
            # nav2_params.inflation_radius = 0.3  # 恢复默认值


def main(args=None):
    """
    主函数
    
    初始化 ROS2 节点并启动任务管理器状态机。
    
    Args:
        args: 命令行参数（可选）
    """
    # 初始化 ROS2
    rclpy.init(args=args)
    
    # 创建任务管理器节点
    node = TaskManagerNode()
    
    try:
        # 运行节点（阻塞直到中断）
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获键盘中断（Ctrl+C）
        pass
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


#!/usr/bin/env python3
"""
Task manager 运算/几何工具函数。

将纯计算逻辑从 task_manager 中抽离，便于测试与复用。
不含 ROS 依赖，仅依赖 geometry_msgs 与 math。
"""

import math
import geometry_msgs.msg as geometry_msgs


def is_pose_in_blacklist(position, object_blacklist, blacklist_radius):
    """
    判断某位置是否落在黑名单区域内。

    Args:
        position: 待检查位置，需有 .x, .y 属性
        object_blacklist: 黑名单位置列表，每项需有 .x, .y
        blacklist_radius: 黑名单半径 (m)

    Returns:
        bool: 若在任一黑名单点半径内返回 True，否则 False
    """
    for blacklist_pos in object_blacklist:
        distance = math.sqrt(
            (position.x - blacklist_pos.x) ** 2
            + (position.y - blacklist_pos.y) ** 2
        )
        if distance < blacklist_radius:
            return True
    return False


def compute_pregrasp_pose(target_pose, distance, robot_x, robot_y,
                          frame_id='map', stamp=None):
    """
    计算预抓取/预放置位姿。

    Args:
        target_pose: geometry_msgs.msg.PoseStamped，目标位姿
        distance: float，站立距离 (m)
        robot_x, robot_y: 机器人当前 x,y (map 系)
        frame_id: 输出位姿的 frame_id
        stamp: 可选，header.stamp

    Returns:
        geometry_msgs.msg.PoseStamped: 预抓取/预放置位姿
    """
    goal_pose = geometry_msgs.PoseStamped()
    goal_pose.header.frame_id = frame_id
    if stamp is not None:
        goal_pose.header.stamp = stamp

    dx = robot_x - target_pose.pose.position.x
    dy = robot_y - target_pose.pose.position.y
    dist = math.sqrt(dx * dx + dy * dy)

    if dist > 0:
        dx /= dist
        dy /= dist
    else:
        dx = 1.0
        dy = 0.0

    goal_pose.pose.position.x = target_pose.pose.position.x + distance * dx
    goal_pose.pose.position.y = target_pose.pose.position.y + distance * dy
    goal_pose.pose.position.z = 0.0

    yaw = math.atan2(
        target_pose.pose.position.y - goal_pose.pose.position.y,
        target_pose.pose.position.x - goal_pose.pose.position.x,
    )
    goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

    return goal_pose

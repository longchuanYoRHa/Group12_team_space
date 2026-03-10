#!/usr/bin/env python3
"""
Task manager utils functions.

Separate pure computation logic from task_manager for testing and reuse.
No ROS dependencies, only depends on geometry_msgs and math.
"""

import math
import geometry_msgs.msg as geometry_msgs


def is_pose_in_blacklist(position, object_blacklist, blacklist_radius):
    """
    Check if a position is in the blacklist.

    Used to avoid repeated attempts at previously failed grasp positions.

    Args:
        position: geometry_msgs.msg.Point, position to check
        object_blacklist: list of geometry_msgs.msg.Point, blacklist positions
        blacklist_radius: float, blacklist radius (m)

    Returns:
        bool: True if within blacklist radius, else False
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
    Compute pregrasp/preplace pose.

    Generate a pose in front of target at given distance, facing target.
    Used for object grasp and bin place navigation.

    Args:
        target_pose: geometry_msgs.msg.PoseStamped, target pose (map frame)
        distance: float, standoff distance (m)
        robot_x: float, robot current x (map frame)
        robot_y: float, robot current y (map frame)
        frame_id: str, output pose frame_id, default 'map'
        stamp: optional, header.stamp, if None then not set

    Returns:
        geometry_msgs.msg.PoseStamped: pregrasp/preplace pose (only position and orientation are valid)
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

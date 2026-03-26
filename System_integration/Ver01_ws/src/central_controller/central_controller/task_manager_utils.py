#!/usr/bin/env python3
"""
Task manager utils functions.

Separate pure computation logic from task_manager for testing and reuse.
No ROS dependencies, only depends on geometry_msgs and math.
"""

import math
import geometry_msgs.msg as geometry_msgs


def quat_yaw(q: geometry_msgs.Quaternion) -> float:
    """Return yaw (rad) from a quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(a: float) -> float:
    """Normalize angle to (-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quaternion_from_yaw(yaw: float) -> geometry_msgs.Quaternion:
    """Create a quaternion representing yaw rotation (rad)."""
    half = yaw * 0.5
    q = geometry_msgs.Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


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
                          frame_id='map', stamp=None, yaw_offset=0.0):
    """
    Compute pregrasp/preplace pose.

    Generate a pose in front of target at given distance, facing target
    (yaw from goal toward target), plus optional yaw_offset (e.g. math.pi
    to flip heading 180° for vision-triggered goals).

    Args:
        target_pose: geometry_msgs.msg.PoseStamped, target pose (map frame)
        distance: float, standoff distance (m)
        robot_x: float, robot current x (map frame)
        robot_y: float, robot current y (map frame)
        frame_id: str, output pose frame_id, default 'map'
        stamp: optional, header.stamp, if None then not set
        yaw_offset: rad, added to computed facing-target yaw (default 0)

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
    yaw = normalize_angle(yaw + yaw_offset)
    goal_pose.pose.orientation = quaternion_from_yaw(yaw)

    return goal_pose

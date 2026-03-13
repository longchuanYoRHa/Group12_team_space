#!/usr/bin/env python3
"""
Mock arm controller node (for integration testing without hardware).

目标：
- 接口对齐 `my_cobot_control/mycobot_controller.py` 的最小子集
- 订阅：  target_pick  (geometry_msgs/Point)  —— 在命名空间 arm 下即 /arm/target_pick
         target_place (geometry_msgs/Point)  —— 在命名空间 arm 下即 /arm/target_place
- 发布：  status         (std_msgs/String) —— 状态机字符串（idle/holding 等）
         gripper_status (std_msgs/String) —— 抓取/放置结果（grip_ok/released 等）

行为：
- 收到 target_pick 且当前 idle：等待 pick_duration_sec -> 进入 holding（模拟抓取成功）
- 收到 target_place 且当前 holding：等待 place_duration_sec -> 回到 idle（模拟放置成功）

说明：
- 本节点不做任何运动学计算，只用于给上层流程提供“成功信号”。
- 推荐以 namespace=arm 启动，以与真实机械臂控制节点一致。
"""

from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import String


class ArmState(Enum):
    IDLE = auto()
    PICKING = auto()
    HOLDING = auto()
    PLACING = auto()


class MockArmController(Node):
    def __init__(self):
        super().__init__('mock_arm_controller')

        self.declare_parameter('pick_duration_sec', 2.0)
        self.declare_parameter('place_duration_sec', 2.0)
        self.declare_parameter('status_publish_hz', 2.0)

        self._state = ArmState.IDLE
        self._last_status_text: Optional[str] = None
        self._last_gripper_text: Optional[str] = None

        self._last_pick_target: Optional[Point] = None
        self._last_place_target: Optional[Point] = None

        self.status_pub = self.create_publisher(String, 'status', 10)
        self.gripper_pub = self.create_publisher(String, 'gripper_status', 10)

        self.create_subscription(Point, 'target_pick', self._on_pick_target, 10)
        self.create_subscription(Point, 'target_place', self._on_place_target, 10)

        hz = float(self.get_parameter('status_publish_hz').value)
        period = 0.5 if hz <= 0.0 else 1.0 / hz
        self._status_timer = self.create_timer(period, self._publish_status_periodic)

        self._publish_status(force=True)
        self.get_logger().info('Mock arm controller ready. Waiting for /arm/target_pick & /arm/target_place.')

    # ----------------------------
    # Sub callbacks
    # ----------------------------
    def _on_pick_target(self, msg: Point):
        if self._state != ArmState.IDLE:
            self.get_logger().info(f'Ignoring target_pick (state={self._state.name})')
            return

        self._last_pick_target = msg
        self._state = ArmState.PICKING
        self._publish_status(force=True)

        dt = float(self.get_parameter('pick_duration_sec').value)
        self.get_logger().info(
            f'Received target_pick: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}). '
            f'Simulating pick success in {dt:.1f}s.'
        )
        self.create_timer(max(dt, 0.0), self._finish_pick_once)

    def _on_place_target(self, msg: Point):
        if self._state != ArmState.HOLDING:
            self.get_logger().info(f'Ignoring target_place (state={self._state.name})')
            return

        self._last_place_target = msg
        self._state = ArmState.PLACING
        self._publish_status(force=True)

        dt = float(self.get_parameter('place_duration_sec').value)
        self.get_logger().info(
            f'Received target_place: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}). '
            f'Simulating place success in {dt:.1f}s.'
        )
        self.create_timer(max(dt, 0.0), self._finish_place_once)

    # ----------------------------
    # State transitions (one-shot)
    # ----------------------------
    def _finish_pick_once(self):
        # 由于 create_timer 是周期定时器，这里做一次性保护
        if self._state != ArmState.PICKING:
            return

        self._state = ArmState.HOLDING
        self._publish_gripper('grip_ok')
        self._publish_status(force=True)

        t = self._last_pick_target
        if t is not None:
            self.get_logger().info(
                f'[MOCK] PICK succeeded at ({t.x:.3f}, {t.y:.3f}, {t.z:.3f}). Now HOLDING.'
            )
        else:
            self.get_logger().info('[MOCK] PICK succeeded. Now HOLDING.')

    def _finish_place_once(self):
        if self._state != ArmState.PLACING:
            return

        self._state = ArmState.IDLE
        self._publish_gripper('released')
        self._publish_status(force=True)

        t = self._last_place_target
        if t is not None:
            self.get_logger().info(
                f'[MOCK] PLACE succeeded at ({t.x:.3f}, {t.y:.3f}, {t.z:.3f}). Now IDLE.'
            )
        else:
            self.get_logger().info('[MOCK] PLACE succeeded. Now IDLE.')

    # ----------------------------
    # Publishers
    # ----------------------------
    def _state_to_text(self) -> str:
        # 与真实 mycobot_controller 的输出风格保持“可读、可监控”
        if self._state == ArmState.IDLE:
            return 'idle'
        if self._state == ArmState.PICKING:
            return 'moving_to_pick'
        if self._state == ArmState.HOLDING:
            return 'holding'
        if self._state == ArmState.PLACING:
            return 'moving_to_place'
        return 'unknown'

    def _publish_status_periodic(self):
        self._publish_status(force=False)

    def _publish_status(self, force: bool):
        text = self._state_to_text()
        if (not force) and (self._last_status_text == text):
            return
        self._last_status_text = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _publish_gripper(self, text: str):
        if self._last_gripper_text == text:
            return
        self._last_gripper_text = text
        msg = String()
        msg.data = text
        self.gripper_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


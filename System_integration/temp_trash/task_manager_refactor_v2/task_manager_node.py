#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from navigation_manager import NavigationManager
from arm_manager import ArmManager
from state_machine import StateMachine

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager_refactor")
        self.nav = NavigationManager(self)
        self.arm = ArmManager(self)
        self.sm = StateMachine(self, self.nav, self.arm)
        self.timer = self.create_timer(0.5, self.loop)

    def loop(self):
        self.sm.tick()

def main():
    rclpy.init()
    node = TaskManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

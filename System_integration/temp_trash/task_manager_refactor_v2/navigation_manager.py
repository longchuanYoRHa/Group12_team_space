class NavigationManager:
    def __init__(self, node):
        self.node = node

    def send_goal(self, pose, description=""):
        self.node.get_logger().info(f"Nav goal: {description} -> {pose}")

    def cancel(self):
        self.node.get_logger().info("Nav goal cancelled")

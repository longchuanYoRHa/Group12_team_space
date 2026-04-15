class ArmManager:
    def __init__(self, node):
        self.node = node
        self.holding = False
        self.busy = False

    def execute(self):
        if self.busy:
            return None
        self.busy = True
        if not self.holding:
            self.node.get_logger().info("Executing PICK")
            self.holding = True
            return "pick"
        else:
            self.node.get_logger().info("Executing PLACE")
            self.holding = False
            return "place"

    def is_done(self):
        if self.busy:
            self.busy = False
            return True
        return False

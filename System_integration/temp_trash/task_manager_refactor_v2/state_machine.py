from models import TaskState, CargoState, Report, BoxInfo
from report_manager import print_report

class StateMachine:
    def __init__(self, node, nav, arm):
        self.node = node
        self.nav = nav
        self.arm = arm
        self.state = TaskState.INIT
        self.cargo = CargoState.EMPTY
        self.report = Report()
        self.detected_colors = set()
        self.interest_points = [(1,1),(2,2)]
        self.ip_index = 0

    def transition(self, new_state):
        self.node.get_logger().info(f"{self.state.value} -> {new_state.value}")
        self.state = new_state

    def tick(self):
        if self.state == TaskState.INIT:
            self.transition(TaskState.PRE_EXPLORE_SCAN)

        elif self.state == TaskState.PRE_EXPLORE_SCAN:
            needed = {"red","green","blue"}
            missing = needed - self.detected_colors
            if missing:
                self.node.get_logger().info(f"Missing {missing}, scanning...")
                self.nav.send_goal("rotate_left_30")
                self.nav.send_goal("rotate_right_60")
            for c in missing:
                self.report.boxes[c] = BoxInfo(color=c, pose=(0,0), detected=False)
            self.transition(TaskState.EXPLORE)

        elif self.state == TaskState.PRECISION_ALIGN:
            self.report.docking_attempts += 1
            self.report.docking_success += 1
            self.transition(TaskState.ARM_EXECUTE)

        elif self.state == TaskState.ARM_EXECUTE:
            action = self.arm.execute()
            if action and self.arm.is_done():
                if action == "pick":
                    self.cargo = CargoState.HAS_OBJECT
                else:
                    self.cargo = CargoState.EMPTY
                    self.report.pick_place_success += 1
                self.transition(TaskState.NAV_TO_INTEREST_POINT)

        elif self.state == TaskState.NAV_TO_INTEREST_POINT:
            if self.ip_index >= len(self.interest_points):
                self.transition(TaskState.FINISHED)
                return
            pt = self.interest_points[self.ip_index]
            self.nav.send_goal(pt, "interest point")
            self.report.interest_points_skipped += 1
            self.ip_index += 1

        elif self.state == TaskState.FINISHED:
            print_report(self.report)

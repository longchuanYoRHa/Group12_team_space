import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pymycobot import MyCobot280, PI_PORT, PI_BAUD
import time
import os
import math

class MockMyCobot280:
    """Hardware mock for testing without actual robot"""
    def __init__(self, port, baud):
        print(f"[MOCK] Initializing MyCobot280 on {port} @ {baud}")
        self.current_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gripper_value = 0
    
    def is_controller_connected(self):
        return 1
    
    def power_on(self):
        print("[MOCK] Power ON")
    
    def set_gripper_state(self, state, speed):
        self.gripper_value = 100 if state == 1 else 0
        print(f"[MOCK] Gripper {'CLOSED' if state == 1 else 'OPEN'} at speed {speed}")
    
    def send_coords(self, coords, speed, mode):
        print(f"[MOCK] Moving to coords {coords}")
        # Simulate inverse kinematics, simplified as assumed joint angles
        self.current_angles = [0.5, -0.3, 0.4, 0.0, 0.5, 0.0]
    
    def send_angles(self, angles, speed):
        print(f"[MOCK] Moving to angles {angles}")
        self.current_angles = angles
    
    def get_angles(self):
        """返回当前关节角度（度）"""
        return [math.degrees(a) for a in self.current_angles]

class PickAndPlaceWithFeedback(Node):
    def __init__(self):
        super().__init__('pick_and_place_with_feedback')
        
        # Publish joint states to RViz
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer: periodically publish current state
        self.feedback_timer = self.create_timer(0.1, self.publish_current_state)
        
        # Initialize hardware or Mock
        use_simulation = not os.path.exists(PI_PORT)
        if use_simulation:
            print("=" * 60)
            print("WARNING: No hardware detected. Running in SIMULATION mode.")
            print("=" * 60)
            self.mc = MockMyCobot280(PI_PORT, PI_BAUD)
        else:
            print("INFO: Hardware detected. Running in REAL mode.")
            self.mc = MyCobot280(PI_PORT, PI_BAUD)
        
        self.joint_names = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
                           'joint5_to_joint4', 'joint6_to_joint5', 'joint6']
        
        # Start pick and place task
        self.get_logger().info('Starting pick and place task...')
        self.run_pick_and_place_async()
    
    def publish_current_state(self):
        """Periodically publish current joint states to RViz"""
        try:
            # Read current angles from hardware (degrees)
            angles_deg = self.mc.get_angles()
            
            # Convert to radians
            import math
            angles_rad = [math.radians(a) for a in angles_deg]
            
            # 发布 JointState
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = angles_rad
            
            self.joint_state_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to get angles: {e}')
    
    def run_pick_and_place_async(self):
        """Asynchronously execute pick and place task"""
        # Use a separate thread to avoid blocking ROS 2 spin
        import threading
        thread = threading.Thread(target=self.run_pick_and_place)
        thread.daemon = True
        thread.start()
    
    def run_pick_and_place(self):
        """Actual pick and place logic"""
        time.sleep(2)  # Wait for initialization
        
        self.get_logger().info('STEP 1: Moving to pick position')
        self.mc.send_angles([0, -30, 45, 0, 30, 0], 50)  # Angles in degrees
        time.sleep(3)
        
        self.get_logger().info('STEP 2: Closing gripper')
        self.mc.set_gripper_state(1, 80)
        time.sleep(2)
        
        self.get_logger().info('STEP 3: Lifting object')
        self.mc.send_angles([0, -20, 30, 0, 20, 0], 50)
        time.sleep(3)
        
        self.get_logger().info('STEP 4: Moving to place position')
        self.mc.send_angles([90, -20, 30, 0, 20, 0], 50)
        time.sleep(3)
        
        self.get_logger().info('STEP 5: Opening gripper')
        self.mc.set_gripper_state(0, 80)
        time.sleep(2)
        
        self.get_logger().info('STEP 6: Returning to home')
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 50)
        time.sleep(3)
        
        self.get_logger().info('Task completed!')

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceWithFeedback()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
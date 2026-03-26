#!/home/student04/robots/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import math
from ultralytics import YOLO

# ---------- SETTINGS ----------
WINDOW_NAME = "Rover Vision: Size Validator"
OV_MODEL_DIR = "/home/student21/Desktop/AERO62520_ws/personal_branch/Group12_team_space/System_integration/Ver01_ws/src/vision_pkg/vision_pkg/best_openvino_model"
CONF_CUTOFF = 0.85
IMG_SIZE = 640
TARGET_CUBE_M = 0.02
BIN_M = 0.20
ERROR_MARGIN = 0.60  
MAX_Z_LIMIT = 0.50

class RoverVisionNode(Node):
    def __init__(self):
        super().__init__('rover_vision_node')
        
        # Mapping labels to topics
        self.pubs = {
            "red_cube":   self.create_publisher(Point, '/target_pick/red', 10),
            "green_cube": self.create_publisher(Point, '/target_pick/green', 10),
            "blue_cube":  self.create_publisher(Point, '/target_pick/blue', 10),
            "red_bin":    self.create_publisher(Point, '/target_place/red', 10),
            "green_bin":  self.create_publisher(Point, '/target_place/green', 10),
            "blue_bin":   self.create_publisher(Point, '/target_place/blue', 10)
        }
        
        # Camera Setup
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        
        self.model = YOLO(OV_MODEL_DIR, task="detect")
        self.prev_time = time.time()
        self.get_logger().info("Vision Node: Logic Synchronized. Ready.")

    def run(self):
        while rclpy.ok():
            try:
                frames = self.pipeline.wait_for_frames(2000)
                aligned = self.align.process(frames)
                depth_f, color_f = aligned.get_depth_frame(), aligned.get_color_frame()
                if not depth_f or not color_f: continue
                
                img = np.asanyarray(color_f.get_data())
                
                curr_time = time.time()
                fps = 1 / (curr_time - self.prev_time) if (curr_time - self.prev_time) > 0 else 0
                self.prev_time = curr_time

                results = self.model.predict(source=img, imgsz=IMG_SIZE, conf=CONF_CUTOFF, device="intel:gpu", verbose=False)

                if results and len(results[0].boxes) > 0:
                    box = results[0].boxes[np.argmax(results[0].boxes.conf.cpu().numpy())]
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    
                    # Get base color from YOLO label (e.g., "red_cube" -> "red")
                    raw_label = self.model.names[int(box.cls)].lower()
                    color_prefix = raw_label.split('_')[0].split(' ')[0] # Handles "red_cube" or "red cube"
                    
                    z = depth_f.get_distance(cx, cy)
                    if 0.0 < z <= MAX_Z_LIMIT:
                        p3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [cx, cy], z)
                        tx, ty, tz = p3d
                        
                        p_l = rs.rs2_deproject_pixel_to_point(self.intrinsics, [float(x1), float(cy)], z)
                        p_r = rs.rs2_deproject_pixel_to_point(self.intrinsics, [float(x2), float(cy)], z)
                        width = math.sqrt(sum([(px - py)**2 for px, py in zip(p_l, p_r)]))

                        # --- SIZE FILTERING LOGIC ---
                        is_cube = (TARGET_CUBE_M * (1-ERROR_MARGIN) <= width <= TARGET_CUBE_M * (1+ERROR_MARGIN))
                        is_bin = (BIN_M * (1-ERROR_MARGIN) <= width <= BIN_M * (1+ERROR_MARGIN))

                        msg = Point(x=float(tx), y=float(ty), z=float(tz))

                        if is_cube:
                            topic_key = f"{color_prefix}_cube"
                            if topic_key in self.pubs:
                                self.pubs[topic_key].publish(msg)
                                self.get_logger().info(f"SUCCESS: {color_prefix.upper()} CUBE (PICK) published")
                                disp_color, txt = (0, 255, 0), f"PICK {color_prefix.upper()}"
                        elif is_bin:
                            topic_key = f"{color_prefix}_bin"
                            if topic_key in self.pubs:
                                self.pubs[topic_key].publish(msg)
                                self.get_logger().info(f"SUCCESS: {color_prefix.upper()} BIN (PLACE) published")
                                disp_color, txt = (255, 100, 0), f"PLACE {color_prefix.upper()}"
                        else:
                            disp_color, txt = (100, 100, 100), "REJECTED (SIZE)"

                        # Visualization
                        cv2.rectangle(img, (x1, y1), (x2, y2), disp_color, 2)
                        cv2.putText(img, txt, (x1, y1-55), 0, 0.6, disp_color, 2)
                        cv2.putText(img, f"X:{tx:+.2f} Y:{ty:+.2f} Z:{tz:.2f}", (x1, y1-35), 0, 0.5, (0, 255, 255), 1)
                        cv2.putText(img, f"SIZE: {width*100:.1f}cm", (x1, y1-15), 0, 0.6, disp_color, 2)

                cv2.putText(img, f"FPS: {int(fps)}", (10, 30), 0, 0.7, (0, 255, 0), 2)
                cv2.imshow(WINDOW_NAME, img)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
                rclpy.spin_once(self, timeout_sec=0)

            except Exception as e:
                self.get_logger().error(f"Loop error: {e}")

    def stop(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = RoverVisionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        print("\n[!] Interrupt clicked, closing vision node and cleaning up...")
    finally:
        node.stop()

if __name__ == '__main__':
    main()

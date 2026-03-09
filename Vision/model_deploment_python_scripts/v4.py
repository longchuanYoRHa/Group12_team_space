import pyrealsense2 as rs
import numpy as np
import cv2
import time
import math
from ultralytics import YOLO

# ---------- SETTINGS ----------
WINDOW_NAME = "Rover Vision: Size Validator"
OV_MODEL_DIR = "best_openvino_model" 
CONF_CUTOFF = 0.90
IMG_SIZE = 640

# Physical Targets (Meters)
TARGET_CUBE_M = 0.02      # 2cm
BIN_M = 0.20              # 20cm
ERROR_MARGIN = 0.60       # 50% Tolerance

# Operational Range
MAX_Z_LIMIT = 0.50      # Ignore everything beyond 0.5m

class RealSenseCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)
        self.intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    def get_frames(self):
        try:
            frames = self.pipeline.wait_for_frames(5000)
            aligned = self.align.process(frames)
            return np.asanyarray(aligned.get_color_frame().get_data()), aligned.get_depth_frame()
        except: return None, None

    def stop(self):
        self.pipeline.stop()

def main():
    model = YOLO(OV_MODEL_DIR, task="detect")
    cam = RealSenseCamera()
    
    # Persistent Memory for sensor blackouts
    last_tx, last_ty = 0.0, 0.0
    prev_time = time.time()

    try:
        while True:
            frame, depth_frame = cam.get_frames()
            if frame is None: continue
            
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
            prev_time = curr_time

            # Inference (Optimised for Intel GPU)
            results = model.predict(source=frame, imgsz=IMG_SIZE, conf=CONF_CUTOFF, 
                                    device="intel:gpu", half=True, verbose=False)

            if results and len(results[0].boxes) > 0:
                # Target the highest confidence detection
                box = results[0].boxes[np.argmax(results[0].boxes.conf.cpu().numpy())]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                
                og_label = model.names[int(box.cls)]
                confidence = float(box.conf)

                # --- 3D & DISTANCE LOGIC ---
                dist_z = depth_frame.get_distance(cx, cy)
                
                # Filter 1: Maximum Distance Rejection
                if dist_z > MAX_Z_LIMIT:
                    continue

                if dist_z > 0.0:
                    # Deproject Center for World Coordinates
                    p3d = rs.rs2_deproject_pixel_to_point(cam.intrinsics, [cx, cy], dist_z)
                    tx, ty, tz = p3d
                    last_tx, last_ty = tx, ty
                    
                    # Calculate Physical Width
                    p_left = rs.rs2_deproject_pixel_to_point(cam.intrinsics, [float(x1), float(cy)], dist_z)
                    p_right = rs.rs2_deproject_pixel_to_point(cam.intrinsics, [float(x2), float(cy)], dist_z)
                    real_w = math.sqrt(sum([(px - py)**2 for px, py in zip(p_left, p_right)]))
                else:
                    # Sensor at 0.00 (Dead zone)
                    tx, ty, tz, real_w = last_tx, last_ty, 0.00, 0.00

                # --- STOP ALARM (Z=0.00) ---
                if tz == 0.00:
                    cv2.rectangle(frame, (0, 0), (640, 100), (0, 0, 255), -1)
                    cv2.putText(frame, "STOP", (210, 75), cv2.FONT_HERSHEY_DUPLEX, 2.5, (255, 255, 255), 7)
                    overlay = frame.copy()
                    cv2.rectangle(overlay, (0,0), (640, 480), (0,0,255), -1)
                    cv2.addWeighted(overlay, 0.15, frame, 0.85, 0, frame)

                # --- SIZE VALIDATION ---
                is_cube = (TARGET_CUBE_M * (1-ERROR_MARGIN) <= real_w <= TARGET_CUBE_M * (1+ERROR_MARGIN))
                is_bin = (BIN_M * (1-ERROR_MARGIN) <= real_w <= BIN_M * (1+ERROR_MARGIN))
                
                if is_cube:
                    final_type = "CUBE"
                    color = (0, 255, 0) # Green
                elif is_bin:
                    final_type = "BIN"
                    color = (255, 100, 0) # Blue-ish
                else:
                    final_type = "REJECTED"
                    color = (100, 100, 100) # Grey

                # --- DRAWING ---
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                # Accuracy and Coordinates labels
                cv2.putText(frame, f"{og_label}: {confidence:.2%}", (x1, y1 - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                cv2.putText(frame, f"X:{tx:+.3f} Y:{ty:+.3f} Z:{tz:.3f}m", (x1, y1 - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(frame, f"SIZE: {real_w*100:.1f}cm ({final_type})", (x1, y1 - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # Dashboard
            cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow(WINDOW_NAME, frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'): 
                break
    finally:
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

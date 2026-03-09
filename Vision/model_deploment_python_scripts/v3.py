import pyrealsense2 as rs
import numpy as np
import cv2
import time
from ultralytics import YOLO

# ---------- Settings ----------
WINDOW_NAME = "YOLO11 + Absolute 3D Metres"
OV_MODEL_DIR = "best_openvino_model" 
CONF_CUTOFF = 0.85
IMG_SIZE = 640

class RealSenseCamera:
    def __init__(self, width=640, height=480):
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
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or color_frame is None: return None, None
            return np.asanyarray(color_frame.get_data()), depth_frame
        except Exception:
            return None, None

    def stop(self):
        self.pipeline.stop()

def main():
    model = YOLO(OV_MODEL_DIR, task="detect")
    cam = RealSenseCamera()
    
    # Persistent Memory
    last_tx, last_ty = 0.0, 0.0
    prev_time = time.time()

    print("🚀 Monitoring... STOP triggers when Z is 0.00. Press 'q' to quit.")

    try:
        while True:
            frame, depth_frame = cam.get_frames()
            if frame is None: continue
                
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
            prev_time = curr_time

            results = model.predict(source=frame, imgsz=IMG_SIZE, conf=CONF_CUTOFF, 
                                    device="intel:gpu", half=True, verbose=False)

            if results and len(results[0].boxes) > 0:
                # Get best detection
                box = results[0].boxes[np.argmax(results[0].boxes.conf.cpu().numpy())]
                
                # Metadata
                label_name = model.names[int(box.cls)]
                confidence = float(box.conf)
                
                # 2D Bbox and Center
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                
                # Get Depth
                dist_z = depth_frame.get_distance(cx, cy)
                
                # Update logic
                if dist_z > 0.0:
                    point_3d = rs.rs2_deproject_pixel_to_point(cam.intrinsics, [cx, cy], dist_z)
                    tx, ty, tz = point_3d
                    last_tx, last_ty = tx, ty 
                else:
                    # SENSOR AT 0.00 (Too close or error)
                    tx, ty, tz = last_tx, last_ty, 0.00

                # --- PERSISTENT STOP LOGIC ---
                # Check for 0.00 every single frame
                if tz == 0.00:
                    # Draw a solid red alert at the top of the frame
                    cv2.rectangle(frame, (0, 0), (640, 100), (0, 0, 255), -1)
                    cv2.putText(frame, "STOP", (210, 75), cv2.FONT_HERSHEY_DUPLEX, 2.5, (255, 255, 255), 7)
                    # Also dim the whole frame slightly to show active alarm state
                    overlay = frame.copy()
                    cv2.rectangle(overlay, (0,0), (640, 480), (0,0,255), -1)
                    cv2.addWeighted(overlay, 0.15, frame, 0.85, 0, frame)

                # --- Draw Detections ---
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                # Accuracy and Coordinates labels
                acc_label = f"{label_name}: {confidence:.2%}"
                coord_label = f"X:{tx:+.3f} Y:{ty:+.3f} Z:{tz:.3f}m"
                
                cv2.putText(frame, acc_label, (x1, y1 - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(frame, coord_label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # FPS and Dashboard
            cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow(WINDOW_NAME, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
    finally:
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

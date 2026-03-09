import pyrealsense2 as rs
import numpy as np
import cv2
import os
from ultralytics import YOLO

# ---------- Settings ----------
WINDOW_NAME = "YOLO11 + RealSense Depth"
OV_MODEL_DIR = "best_openvino_model" 
CONF_CUTOFF = 0.90
IMG_SIZE = 640

class RealSenseCamera:
    def __init__(self, width=1280, height=720):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
        
        self.profile = self.pipeline.start(config)
        # Alignment is CRITICAL: maps depth frame to color frame coordinates
        self.align = rs.align(rs.stream.color)

    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return None, None, None
            
        color_image = np.asanyarray(color_frame.get_data())
        return color_image, depth_frame, color_frame

    def stop(self):
        self.pipeline.stop()

def main():
    model = YOLO(OV_MODEL_DIR, task="detect")
    cam = RealSenseCamera()

    try:
        while True:
            frame, depth_frame, _ = cam.get_frames()
            if frame is None: continue

            results = model.predict(source=frame, imgsz=IMG_SIZE, conf=CONF_CUTOFF, 
                                    device="intel:gpu", half=True, verbose=False)

            if results and len(results[0].boxes) > 0:
                boxes = results[0].boxes
                best_box_idx = np.argmax(boxes.conf.cpu().numpy())
                box = boxes[best_box_idx]
                
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Calculate center of the bounding box
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                
                # Get distance in meters from the ACTUAL depth sensor
                distance_m = depth_frame.get_distance(cx, cy)
                distance_cm = distance_m * 100

                # Draw Overlay
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1) # Center point
                
                label = f"{model.names[int(box.cls[0])]} {float(box.conf):.2f}"
                dist_text = f"Depth: {distance_cm:.1f} cm"
                cv2.putText(frame, f"{label} | {dist_text}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow(WINDOW_NAME, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
    finally:
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

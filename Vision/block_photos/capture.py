import cv2
import os
from pathlib import Path

# 1. Define and create folder
save_dir = Path.home() / "block photos"
save_dir.mkdir(parents=True, exist_ok=True)

# 2. Open USB Camera
cam = cv2.VideoCapture(0)

# SET RESOLUTION HERE
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

img_counter = 00

if not cam.isOpened():
    print("Error: Could not open USB camera.")
    exit()

# Verify resolution (some cameras only support specific presets)
actual_w = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
actual_h = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"Resolution set to: {actual_w} x {actual_h}")
print(f"Saving to: {save_dir}")
print("Controls: ENTER to capture | ESC to quit")

while True:
    ret, frame = cam.read()
    if not ret:
        print("Failed to grab frame")
        break

    cv2.imshow("Linux Camera Preview", frame)

    key = cv2.waitKey(1) & 0xFF
    
    if key == 27: # ESC
        break
    elif key == 13: # ENTER
        img_name = f"img_{img_counter}.jpg"
        file_path = str(save_dir / img_name)
        
        success = cv2.imwrite(file_path, frame)
        if success:
            print(f"Saved: {img_name}")
            img_counter += 1
        else:
            print("Error saving image.")

cam.release()
cv2.destroyAllWindows()
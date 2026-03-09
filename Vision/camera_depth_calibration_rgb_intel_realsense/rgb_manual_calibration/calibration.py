import pyrealsense2 as rs
import cv2
import numpy as np
import sys

# --- 1. CONFIGURATION ---
CHESSBOARD_SIZE = (15, 10)  # 16x11 squares = 15x10 internal corners
SQUARE_SIZE = 17.0          # mm
WIDTH, HEIGHT = 1280, 720
CALIB_FILE = "d435_720p_setup.yml"

# --- 2. DEVICE INITIALISATION ---
pipeline = rs.pipeline()
config = rs.config()

# Search for connected devices to avoid "Couldn't resolve requests"
context = rs.context()
devices = context.query_devices()
if not devices:
    print("No RealSense devices found. Check your USB connection.")
    sys.exit()

# Print found devices for your reference
for i, dev in enumerate(devices):
    print(f"Found Device [{i}]: {dev.get_info(rs.camera_info.name)} (S/N: {dev.get_info(rs.camera_info.serial_number)})")

# Request 1280x720. Note: If using USB 2.0, this may still throw a RuntimeError.
try:
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, 30)
    profile = pipeline.start(config)
except RuntimeError as e:
    print(f"\nERROR: Could not start 720p stream: {e}")
    print("Likely cause: USB 2.0 port/cable. Trying 640x480 fallback...")
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)

# --- 3. CALIBRATION MATH ---
align = rs.align(rs.stream.color)

# Get the specific intrinsics for the current resolution
intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
camera_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                          [0, intrinsics.fy, intrinsics.ppy],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float32)

# World coordinates of chessboard corners
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2) * SQUARE_SIZE

print(f"\nStreaming at {intrinsics.width}x{intrinsics.height}")
print("Press 'S' to SAVE calibration or 'Q' to quit.")

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        if not color_frame: continue

        img = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Find corners
        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

        if ret:
            # Refine
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            
            # Distance Calculation
            success, rvec, tvec = cv2.solvePnP(objp, corners2, camera_matrix, dist_coeffs)
            
            if success:
                distance_mm = np.linalg.norm(tvec)
                cv2.drawChessboardCorners(img, CHESSBOARD_SIZE, corners2, ret)
                cv2.putText(img, f"Dist: {distance_mm:.1f}mm", (40, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)

                # Input Handling
                key = cv2.waitKey(1)
                if key & 0xFF == ord('s'):
                    fs = cv2.FileStorage(CALIB_FILE, cv2.FileStorage_WRITE)
                    fs.write("camera_matrix", camera_matrix)
                    fs.write("dist_coeffs", dist_coeffs)
                    fs.write("rvec", rvec)
                    fs.write("tvec", tvec)
                    fs.release()
                    print(f"SAVED: {CALIB_FILE}")
        
        cv2.imshow('D435 Calibration', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

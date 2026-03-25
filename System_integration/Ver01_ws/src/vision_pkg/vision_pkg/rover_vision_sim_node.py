import math
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

# ---------- SETTINGS ----------
WINDOW_NAME = "Rover Vision (SIM): Size Validator"
DEFAULT_MODEL_DIR = "best_openvino_model"

CONF_CUTOFF = 0.85
IMG_SIZE = 640
TARGET_CUBE_M = 0.02
BIN_M = 0.20
ERROR_MARGIN = 0.60
MAX_Z_LIMIT = 0.70


def _camera_info_to_k(info: CameraInfo) -> Tuple[float, float, float, float]:
    # K = [fx 0 cx 0 fy cy 0 0 1]
    fx = float(info.k[0])
    fy = float(info.k[4])
    cx = float(info.k[2])
    cy = float(info.k[5])
    return fx, fy, cx, cy


def _depth_at(depth: np.ndarray, u: int, v: int) -> float:
    if v < 0 or u < 0 or v >= depth.shape[0] or u >= depth.shape[1]:
        return 0.0

    # Common encodings after bridge:
    # - 32FC1: meters
    # - 16UC1: millimeters
    val = depth[v, u]
    if depth.dtype == np.float32 or depth.dtype == np.float64:
        z = float(val)
    elif depth.dtype == np.uint16:
        z = float(val) / 1000.0
    else:
        z = float(val)

    if not math.isfinite(z):
        return 0.0
    return z


def _deproject(u: float, v: float, z: float, fx: float, fy: float, cx: float, cy: float) -> Tuple[float, float, float]:
    x = (u - cx) / fx * z
    y = (v - cy) / fy * z
    return x, y, z


class RoverVisionSimNode(Node):
    def __init__(self):
        super().__init__("rover_vision_sim_node")

        # Parameters (allow easy topic remap without editing code)
        self.declare_parameter("color_topic", "/D435i_camera/color/image_raw")
        self.declare_parameter("depth_topic", "/D435i_camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/D435i_camera/color/camera_info")
        self.declare_parameter("use_openvino_gpu", True)
        # OpenVINO / Ultralytics: "intel:gpu" -> Intel GPU (通常为核显); 多 GPU 时可试 "intel:gpu.0" / "intel:gpu.1"
        self.declare_parameter("openvino_device", "intel:gpu")
        self.declare_parameter("visualize", True)
        self.declare_parameter("model_dir", "")

        color_topic = self.get_parameter("color_topic").get_parameter_value().string_value
        depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value

        # Output topics: keep identical to real node
        self.pubs = {
            "red_cube": self.create_publisher(Point, "/target_pick/red", 10),
            "green_cube": self.create_publisher(Point, "/target_pick/green", 10),
            "blue_cube": self.create_publisher(Point, "/target_pick/blue", 10),
            "red_bin": self.create_publisher(Point, "/target_place/red", 10),
            "green_bin": self.create_publisher(Point, "/target_place/green", 10),
            "blue_bin": self.create_publisher(Point, "/target_place/blue", 10),
        }

        self.bridge = CvBridge()
        self._last_color: Optional[np.ndarray] = None
        self._last_depth: Optional[np.ndarray] = None
        self._last_stamp = None

        self._fx = self._fy = self._cx = self._cy = None

        self.create_subscription(Image, color_topic, self._on_color, 10)
        self.create_subscription(Image, depth_topic, self._on_depth, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self._on_info, 10)

        model_dir_param = self.get_parameter("model_dir").get_parameter_value().string_value.strip()
        candidates = []
        if model_dir_param:
            candidates.append(Path(model_dir_param))
        candidates.append(Path(__file__).resolve().parent / DEFAULT_MODEL_DIR)
        try:
            candidates.append(Path(get_package_share_directory("vision_pkg")) / DEFAULT_MODEL_DIR)
        except Exception:
            pass

        model_dir = next((p for p in candidates if p.exists()), None)
        if model_dir is None:
            msg = "Model directory not found. Tried: " + ", ".join(str(p) for p in candidates)
            raise FileNotFoundError(msg)

        self.model = YOLO(str(model_dir), task="detect")
        self.prev_time = time.time()

        self.timer = self.create_timer(1.0 / 30.0, self._tick)
        use_ov = self.get_parameter("use_openvino_gpu").get_parameter_value().bool_value
        ov_dev = self.get_parameter("openvino_device").get_parameter_value().string_value.strip() or "intel:gpu"
        self.get_logger().info(
            f"SIM vision subscribed to color={color_topic}, depth={depth_topic}, info={camera_info_topic}; "
            f"OpenVINO device={(ov_dev if use_ov else 'intel:cpu')}"
        )

    def _on_info(self, msg: CameraInfo) -> None:
        fx, fy, cx, cy = _camera_info_to_k(msg)
        if fx > 0.0 and fy > 0.0:
            self._fx, self._fy, self._cx, self._cy = fx, fy, cx, cy

    def _on_color(self, msg: Image) -> None:
        try:
            # Gazebo camera is RGB; bridge may deliver rgb8 or bgr8
            if msg.encoding.lower() in ("rgb8", "rgba8"):
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._last_color = img
            self._last_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"Color convert failed: {e}")

    def _on_depth(self, msg: Image) -> None:
        try:
            # Keep raw depth values; cv_bridge will map 16UC1/32FC1 accordingly
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self._last_depth = depth
        except Exception as e:
            self.get_logger().error(f"Depth convert failed: {e}")

    def _tick(self) -> None:
        if self._last_color is None or self._last_depth is None:
            return
        if self._fx is None:
            # Without CameraInfo we cannot reliably deproject; wait.
            return

        img = self._last_color.copy()
        depth = self._last_depth

        curr_time = time.time()
        fps = 1 / (curr_time - self.prev_time) if (curr_time - self.prev_time) > 0 else 0
        self.prev_time = curr_time

        use_openvino_gpu = self.get_parameter("use_openvino_gpu").get_parameter_value().bool_value
        ov_device = self.get_parameter("openvino_device").get_parameter_value().string_value.strip()
        if not ov_device:
            ov_device = "intel:gpu"
        # OpenVINO IR 模型用 intel:*；非 GPU 时走 OpenVINO CPU 插件（勿用 "cpu"，否则可能走错后端）
        device = ov_device if use_openvino_gpu else "intel:cpu"

        try:
            results = self.model.predict(
                source=img,
                imgsz=IMG_SIZE,
                conf=CONF_CUTOFF,
                device=device,
                verbose=False,
            )
        except Exception as e:
            self.get_logger().error(f"YOLO predict failed: {e}")
            return

        if results and len(results[0].boxes) > 0:
            box = results[0].boxes[np.argmax(results[0].boxes.conf.cpu().numpy())]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            raw_label = self.model.names[int(box.cls)].lower()
            color_prefix = raw_label.split("_")[0].split(" ")[0]

            z = _depth_at(depth, cx, cy)
            if 0.0 < z <= MAX_Z_LIMIT:
                fx, fy, cxo, cyo = float(self._fx), float(self._fy), float(self._cx), float(self._cy)
                tx, ty, tz = _deproject(float(cx), float(cy), z, fx, fy, cxo, cyo)

                # Estimate object width at same depth using left/right points on bbox centerline
                xl, yl, _ = _deproject(float(x1), float(cy), z, fx, fy, cxo, cyo)
                xr, yr, _ = _deproject(float(x2), float(cy), z, fx, fy, cxo, cyo)
                width = math.sqrt((xl - xr) ** 2 + (yl - yr) ** 2)

                is_cube = (TARGET_CUBE_M * (1 - ERROR_MARGIN) <= width <= TARGET_CUBE_M * (1 + ERROR_MARGIN))
                is_bin = (BIN_M * (1 - ERROR_MARGIN) <= width <= BIN_M * (1 + ERROR_MARGIN))

                msg = Point(x=float(tx), y=float(ty), z=float(tz))

                disp_color = (100, 100, 100)
                txt = "REJECTED (SIZE)"

                if is_cube:
                    topic_key = f"{color_prefix}_cube"
                    if topic_key in self.pubs:
                        self.pubs[topic_key].publish(msg)
                        disp_color, txt = (0, 255, 0), f"PICK {color_prefix.upper()}"
                elif is_bin:
                    topic_key = f"{color_prefix}_bin"
                    if topic_key in self.pubs:
                        self.pubs[topic_key].publish(msg)
                        disp_color, txt = (255, 100, 0), f"PLACE {color_prefix.upper()}"

                if self.get_parameter("visualize").get_parameter_value().bool_value:
                    cv2.rectangle(img, (x1, y1), (x2, y2), disp_color, 2)
                    cv2.putText(img, txt, (x1, y1 - 55), 0, 0.6, disp_color, 2)
                    cv2.putText(img, f"X:{tx:+.2f} Y:{ty:+.2f} Z:{tz:.2f}", (x1, y1 - 35), 0, 0.5, (0, 255, 255), 1)
                    cv2.putText(img, f"SIZE: {width * 100:.1f}cm", (x1, y1 - 15), 0, 0.6, disp_color, 2)

        if self.get_parameter("visualize").get_parameter_value().bool_value:
            cv2.putText(img, f"FPS: {int(fps)}", (10, 30), 0, 0.7, (0, 255, 0), 2)
            cv2.imshow(WINDOW_NAME, img)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = RoverVisionSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
将视觉话题中的相机系坐标（vision_x/y/z）与变换后的 map 坐标写入同一 CSV，并记录 PGM 兴趣点（无视觉列）。

视觉坐标与 task_manager 中一致：geometry_msgs/Point，frame 为节点的 camera_frame_id（通常为相机光学系）。
"""

from __future__ import annotations

import csv
import math
import os
import threading
from datetime import datetime, timezone
from typing import Iterable, List, Optional, Tuple

FIELDNAMES = (
    "timestamp_iso",
    "event_type",
    "color",
    "vision_x",
    "vision_y",
    "vision_z",
    "map_x",
    "map_y",
    "note",
)


def _cell_float(v: Optional[float]) -> str:
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return ""
    return f"{float(v):.6f}"


class DetectionMapCoordinatesCsvLogger:
    """线程安全的单行追加 CSV 记录器。"""

    def __init__(self, filepath: str):
        self._path = os.path.abspath(filepath)
        self._lock = threading.Lock()
        self._ensure_header()

    def _ensure_header(self) -> None:
        d = os.path.dirname(self._path)
        if d:
            os.makedirs(d, exist_ok=True)
        need_header = not os.path.isfile(self._path) or os.path.getsize(self._path) == 0
        if need_header:
            with open(self._path, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=FIELDNAMES)
                w.writeheader()

    def _append(self, row: dict) -> None:
        def cell_coord(key: str) -> str:
            v = row.get(key, "")
            if v == "" or v is None:
                return ""
            if isinstance(v, (int, float)):
                return _cell_float(float(v))
            return str(v)

        rec = {
            "timestamp_iso": row.get("timestamp_iso", ""),
            "event_type": row.get("event_type", ""),
            "color": row.get("color", ""),
            "vision_x": cell_coord("vision_x"),
            "vision_y": cell_coord("vision_y"),
            "vision_z": cell_coord("vision_z"),
            "map_x": cell_coord("map_x"),
            "map_y": cell_coord("map_y"),
            "note": row.get("note", ""),
        }
        with self._lock:
            with open(self._path, "a", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=FIELDNAMES, extrasaction="ignore")
                w.writerow(rec)
                f.flush()

    @staticmethod
    def _ts() -> str:
        return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")

    def log_object_map_nav(
        self,
        color: str,
        vision_x: float,
        vision_y: float,
        vision_z: float,
        map_x: float,
        map_y: float,
        task_state: str,
    ) -> None:
        """稳定检测后触发导航至物体预抓取位时调用。"""
        self._append(
            {
                "timestamp_iso": self._ts(),
                "event_type": "object_map_nav",
                "color": color,
                "vision_x": vision_x,
                "vision_y": vision_y,
                "vision_z": vision_z,
                "map_x": map_x,
                "map_y": map_y,
                "note": f"task_state={task_state}",
            }
        )

    def log_object_pick_arm(
        self,
        color: str,
        vision_x: float,
        vision_y: float,
        vision_z: float,
        map_x: Optional[float],
        map_y: Optional[float],
    ) -> None:
        """GRASP 状态下发送 /arm/target_pick 时：相机系点 + 最近一次 object 的 map 平面坐标（若有）。"""
        self._append(
            {
                "timestamp_iso": self._ts(),
                "event_type": "object_pick_arm",
                "color": color,
                "vision_x": vision_x,
                "vision_y": vision_y,
                "vision_z": vision_z,
                "map_x": map_x,
                "map_y": map_y,
                "note": "",
            }
        )

    def log_bin_map_cached(
        self,
        color: str,
        vision_x: float,
        vision_y: float,
        vision_z: float,
        map_x: float,
        map_y: float,
        task_state: str,
    ) -> None:
        """探索 / 预旋转阶段仅缓存 bin map 位姿时调用。"""
        self._append(
            {
                "timestamp_iso": self._ts(),
                "event_type": "bin_map_cached",
                "color": color,
                "vision_x": vision_x,
                "vision_y": vision_y,
                "vision_z": vision_z,
                "map_x": map_x,
                "map_y": map_y,
                "note": f"task_state={task_state}",
            }
        )

    def log_bin_map_nav(
        self,
        color: str,
        vision_x: float,
        vision_y: float,
        vision_z: float,
        map_x: float,
        map_y: float,
        task_state: str,
    ) -> None:
        """稳定检测后触发导航至 bin 预放置位时调用。"""
        self._append(
            {
                "timestamp_iso": self._ts(),
                "event_type": "bin_map_nav",
                "color": color,
                "vision_x": vision_x,
                "vision_y": vision_y,
                "vision_z": vision_z,
                "map_x": map_x,
                "map_y": map_y,
                "note": f"task_state={task_state}",
            }
        )

    def log_bin_map_place_command(
        self,
        color: str,
        vision_x: Optional[float],
        vision_y: Optional[float],
        vision_z: Optional[float],
        map_x: float,
        map_y: float,
    ) -> None:
        """PLACE_IN_BIN 下发放置目标时：map 坐标 + 最近一次视觉点（若可用）。"""
        self._append(
            {
                "timestamp_iso": self._ts(),
                "event_type": "bin_map_place_command",
                "color": color,
                "vision_x": vision_x,
                "vision_y": vision_y,
                "vision_z": vision_z,
                "map_x": map_x,
                "map_y": map_y,
                "note": "",
            }
        )

    def log_pgm_points(
        self,
        points: Iterable[Tuple[float, float]],
        event_type: str,
        *,
        pgm_path: str = "",
    ) -> None:
        """
        记录 PGM 检测得到的一系列 map 坐标（米）。
        event_type 使用 'pgm_raw' 或 'pgm_filtered'。
        """
        pts: List[Tuple[float, float]] = list(points)
        n = len(pts)
        base_note = f"count={n}"
        if pgm_path:
            base_note = f"{base_note};pgm={pgm_path}"
        for i, (mx, my) in enumerate(pts):
            self._append(
                {
                    "timestamp_iso": self._ts(),
                    "event_type": event_type,
                    "color": "",
                    "vision_x": "",
                    "vision_y": "",
                    "vision_z": "",
                    "map_x": mx,
                    "map_y": my,
                    "note": f"{base_note};index={i}",
                }
            )

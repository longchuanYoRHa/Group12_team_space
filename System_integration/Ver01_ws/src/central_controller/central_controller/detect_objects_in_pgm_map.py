#!/usr/bin/env python3
"""
PGM 地图封闭空间内物体点位识别样例程序（独立脚本，非 ROS 节点）

直接运行测试：
  cd real_Nav_test && python3 real_Nav_test/detect_objects_in_pgm_map.py
  python3 real_Nav_test/detect_objects_in_pgm_map.py maps/tb3_sandbox.pgm
  python3 real_Nav_test/detect_objects_in_pgm_map.py --mode enclosed-blobs --help

两种模式：
  default     最外侧封闭轮廓内的「可通行房间」→ 房间内占据物（盒子/台子）中心。
  enclosed-blobs  白色中间的灰/黑(205+占据)四连通区域；不按像素面积，按包围盒在地图坐标系下
  x、y 方向跨度均不超过给定米数(默认 0.30m)；触边时仍要求边界邻接白色比例。

像素约定（与 map_server 一致）：0=可通行(白)，254/255=占据，205=未知(深灰等)。
"""

from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path
from typing import List, Tuple, Optional

try:
    from PIL import Image

    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False

try:
    import yaml  # type: ignore
    _HAS_YAML = True
except ImportError:
    _HAS_YAML = False

# 地图语义（与 map_server 一致）
FREE = 0
# 占据：部分地图保存为 254 或 255，均视为障碍/物体
OCCUPIED_VALUES = (254, 255)
UNKNOWN = 205


def _is_occupied(pixel_val: int) -> bool:
    return pixel_val in OCCUPIED_VALUES


def connected_components_by_predicate(
    w: int, h: int, pixels: List[int], pred
) -> List[List[int]]:
    """对满足 pred(pixel) 的像素做 4-连通分量。"""
    n = w * h
    visited = [False] * n
    components: List[List[int]] = []
    for start in range(n):
        if visited[start] or not pred(pixels[start]):
            continue
        comp: List[int] = []
        q = deque([start])
        visited[start] = True
        while q:
            i = q.popleft()
            comp.append(i)
            for j in pixel_neighbors_4(w, h, i):
                if not visited[j] and pred(pixels[j]):
                    visited[j] = True
                    q.append(j)
        components.append(comp)
    return components


def bbox_inclusive_spans_px(comp: List[int], w: int) -> Tuple[int, int]:
    """轴对齐包围盒在 x、y 方向的像素跨度（含端点像素）。"""
    min_x = min_y = 10**9
    max_x = max_y = -1
    for idx in comp:
        y, x = divmod(idx, w)
        if x < min_x:
            min_x = x
        if x > max_x:
            max_x = x
        if y < min_y:
            min_y = y
        if y > max_y:
            max_y = y
    return (max_x - min_x + 1), (max_y - min_y + 1)


def component_lies_entirely_on_one_map_edge(comp: List[int], w: int, h: int) -> bool:
    """整块像素全落在图像某一条外缘上（底边一条未知、左边一列噪声等），不作为兴趣点。"""
    xs: List[int] = []
    ys: List[int] = []
    for idx in comp:
        y, x = divmod(idx, w)
        xs.append(x)
        ys.append(y)
    return (
        all(x == 0 for x in xs)
        or all(x == w - 1 for x in xs)
        or all(y == 0 for y in ys)
        or all(y == h - 1 for y in ys)
    )


def dedupe_interest_blobs(
    blobs: List[Tuple[float, float, int]], min_separation_px: float
) -> List[Tuple[float, float, int]]:
    """质心过近时保留像素数更多的块，避免同一目标被 205/占据拆成两个点。"""
    if min_separation_px <= 0 or not blobs:
        return sorted(blobs, key=lambda b: (-b[1], b[0]))
    blobs_by_area = sorted(blobs, key=lambda b: -b[2])
    kept: List[Tuple[float, float, int]] = []
    sep2 = min_separation_px * min_separation_px
    for b in blobs_by_area:
        bx, by, _ = b
        if all((bx - kx) ** 2 + (by - ky) ** 2 >= sep2 for kx, ky, _ in kept):
            kept.append(b)
    kept.sort(key=lambda b: (-b[1], b[0]))
    return kept


# 默认分辨率 (m/pixel)，与 mapper_params 一致；若使用 map.yaml 请从 yaml 读取
DEFAULT_RESOLUTION = 0.05
# 默认地图原点 (m)，若有 map.yaml 的 origin 请替换
DEFAULT_ORIGIN = (0.0, 0.0)

# 物体占位过滤：像素块面积范围（像素数），过滤噪声与过大的墙
MIN_OBJECT_PIXELS = 10
MAX_OBJECT_PIXELS = 2500


def load_map_metadata_from_yaml(pgm_path: str) -> Optional[Tuple[float, Tuple[float, float]]]:
    """
    从与 pgm 同目录、同名的 map yaml 读取 (resolution, (origin_x, origin_y))。
    注意：PGM 本身不包含原点信息，origin/resolution 只存在于 YAML。
    """
    yaml_path = str(Path(pgm_path).with_suffix(".yaml"))
    if not Path(yaml_path).is_file():
        return None

    try:
        if _HAS_YAML:
            with open(yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            res = float(data.get("resolution"))
            origin = data.get("origin")
            ox = float(origin[0])
            oy = float(origin[1])
            return res, (ox, oy)
        # 无 PyYAML 时：做一个很窄的解析，只支持本项目的简单 yaml 行
        res = None
        origin = None
        with open(yaml_path, "r", encoding="utf-8") as f:
            for line in f:
                s = line.strip()
                if not s or s.startswith("#"):
                    continue
                if s.startswith("resolution:"):
                    res = float(s.split(":", 1)[1].strip())
                elif s.startswith("origin:"):
                    rhs = s.split(":", 1)[1].strip()
                    rhs = rhs.strip().lstrip("[").rstrip("]")
                    parts = [p.strip() for p in rhs.split(",") if p.strip()]
                    if len(parts) >= 2:
                        origin = (float(parts[0]), float(parts[1]))
        if res is None or origin is None:
            return None
        return float(res), (float(origin[0]), float(origin[1]))
    except Exception:
        return None


def load_pgm(path: str) -> Tuple[int, int, List[int]]:
    """加载 P5 或 P2 格式 PGM，返回 (width, height, 一维像素列表)。"""
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic == b"P5":
            # 跳过注释行
            while True:
                line = f.readline()
                if not line.startswith(b"#"):
                    break
            parts = line.decode().split()
            width, height = int(parts[0]), int(parts[1])
            maxval = int(f.readline().decode().strip())
            data = f.read()
            pixels = list(data) if maxval <= 255 else []
            if len(pixels) != width * height and maxval > 255:
                # 16-bit
                import struct

                count = width * height
                pixels = list(struct.unpack(f">{count}H", data[: count * 2]))
            return width, height, pixels
        elif magic == b"P2":
            while True:
                line = f.readline()
                if not line.startswith(b"#"):
                    break
            parts = line.decode().split()
            width, height = int(parts[0]), int(parts[1])
            maxval = int(f.readline().decode().strip())
            rest = f.read().decode().replace("\n", " ").split()
            pixels = [int(x) for x in rest if x.strip()]
            return width, height, pixels
    raise ValueError(f"Unsupported PGM format: {magic}")


def pixel_neighbors_4(w: int, h: int, idx: int) -> List[int]:
    """返回 4 邻域在扁平数组中的索引（不越界）。"""
    y, x = divmod(idx, w)
    out = []
    for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        ny, nx = y + dy, x + dx
        if 0 <= ny < h and 0 <= nx < w:
            out.append(ny * w + nx)
    return out


def connected_components(
    w: int, h: int, pixels: List[int], foreground_val: int
) -> List[List[int]]:
    """对像素值为 foreground_val 的像素做连通分量，返回每个分量的一维索引列表。"""
    n = w * h
    visited = [False] * n
    components = []
    for start in range(n):
        if visited[start] or pixels[start] != foreground_val:
            continue
        comp = []
        q = deque([start])
        visited[start] = True
        while q:
            i = q.popleft()
            comp.append(i)
            for j in pixel_neighbors_4(w, h, i):
                if not visited[j] and pixels[j] == foreground_val:
                    visited[j] = True
                    q.append(j)
        components.append(comp)
    return components


def connected_components_non_free(w: int, h: int, pixels: List[int]) -> List[List[int]]:
    """对「非白色」(非 FREE) 像素做连通分量，用于识别被白色包裹的深色区域。"""
    n = w * h
    visited = [False] * n
    components = []
    for start in range(n):
        if visited[start] or pixels[start] == FREE:
            continue
        comp = []
        q = deque([start])
        visited[start] = True
        while q:
            i = q.popleft()
            comp.append(i)
            for j in pixel_neighbors_4(w, h, i):
                if not visited[j] and pixels[j] != FREE:
                    visited[j] = True
                    q.append(j)
        components.append(comp)
    return components


def touches_border(w: int, h: int, indices: List[int]) -> bool:
    """连通分量是否触及图像边界（则视为非封闭）。"""
    for idx in indices:
        y, x = divmod(idx, w)
        if y == 0 or y == h - 1 or x == 0 or x == w - 1:
            return True
    return False


def boundary_white_ratio(w: int, h: int, pixels: List[int], comp: List[int]) -> float:
    """
    区块边界中邻接白色(FREE)的比例。1.0=完全被白色包围，0=完全不邻接白色。
    用于判定「白色中间有明显区块」：比例越高越像物体。
    """
    comp_set = set(comp)
    edges_white = 0
    edges_any = 0
    for idx in comp:
        for j in pixel_neighbors_4(w, h, idx):
            if j not in comp_set:
                edges_any += 1
                if pixels[j] == FREE:
                    edges_white += 1
    if edges_any == 0:
        return 0.0
    return edges_white / edges_any


def centroid_pixel(indices: List[int], w: int) -> Tuple[float, float]:
    """分量质心（像素坐标），(x, y) 图像坐标系。"""
    if not indices:
        return 0.0, 0.0
    sx = sy = 0.0
    for idx in indices:
        y, x = divmod(idx, w)
        sx += x
        sy += y
    n = len(indices)
    return sx / n, sy / n


def pixel_to_map(
    px: float,
    py: float,
    resolution: float,
    origin_x: float,
    origin_y: float,
    height: int,
) -> Tuple[float, float]:
    """像素坐标转地图坐标 (m)。图像 y 向下，地图 y 向上时需翻转。"""
    # 常见约定：图像 (0,0) 在左上，地图原点在左下，故 mx = ox + px*res, my = oy + (height-1-py)*res
    mx = origin_x + px * resolution
    my = origin_y + (height - 1 - py) * resolution
    return mx, my


def draw_result_image(
    w: int,
    h: int,
    pixels: List[int],
    points_px: List[Tuple[float, float, int]],
    output_path: str,
    color: Tuple[int, int, int] = (255, 0, 0),
) -> None:
    """
    根据栅格数据生成灰度底图，在 points_px 几何中心处点一个红色格，保存为 PNG。
    points_px: [(cx_px, cy_px, area), ...]
    """
    if not _HAS_PIL:
        print("未安装 Pillow，跳过生成标注图。可执行: pip install Pillow")
        return
    img = Image.new("L", (w, h))
    img.putdata(pixels)
    rgb = img.convert("RGB")
    for cx, cy, _ in points_px:
        ix, iy = int(round(cx)), int(round(cy))
        if 0 <= ix < w and 0 <= iy < h:
            rgb.putpixel((ix, iy), color)
    rgb.save(output_path)
    print(f"已保存标注图: {output_path}")


def find_enclosed_free_regions(
    w: int, h: int, pixels: List[int], include_unknown: bool = False
) -> List[List[int]]:
    """
    找出所有不接触边界的可通行连通区域，视为封闭空间。
    include_unknown=True 时，将未知(205)也视为可通行，用于在未完全建图区域中检测物体。
    """
    if not include_unknown:
        free_components = connected_components(w, h, pixels, FREE)
    else:
        # 可通行 = FREE 或 UNKNOWN
        n = w * h
        visited = [False] * n
        free_components = []
        for start in range(n):
            if visited[start] or (pixels[start] != FREE and pixels[start] != UNKNOWN):
                continue
            comp = []
            q = deque([start])
            visited[start] = True
            while q:
                i = q.popleft()
                comp.append(i)
                for j in pixel_neighbors_4(w, h, i):
                    if not visited[j] and (pixels[j] == FREE or pixels[j] == UNKNOWN):
                        visited[j] = True
                        q.append(j)
            free_components.append(comp)
    enclosed = [c for c in free_components if not touches_border(w, h, c)]
    return enclosed


def occupied_blobs_adjacent_to_free_region(
    w: int,
    h: int,
    pixels: List[int],
    free_region: set,
    min_pixels: int,
    max_pixels: int,
) -> List[Tuple[float, float, int]]:
    """
    找出与 free_region 相邻的 OCCUPIED 连通块（即该封闭空间内的物体），
    返回 (cx_px, cy_px, area) 列表，面积在 [min_pixels, max_pixels] 内。
    """
    # 所有与 free_region 相邻的占据像素
    seed = set()
    for i in free_region:
        for j in pixel_neighbors_4(w, h, i):
            if _is_occupied(pixels[j]):
                seed.add(j)
    if not seed:
        return []
    # 在 seed 内做连通分量（仅考虑占据且与 free_region 邻接的像素之间的邻接）
    visited = set()
    components = []
    for start in seed:
        if start in visited:
            continue
        comp = []
        q = deque([start])
        visited.add(start)
        while q:
            i = q.popleft()
            comp.append(i)
            for j in pixel_neighbors_4(w, h, i):
                if j not in visited and _is_occupied(pixels[j]):
                    visited.add(j)
                    q.append(j)
        components.append(comp)
    result = []
    for comp in components:
        area = len(comp)
        if min_pixels <= area <= max_pixels:
            cx, cy = centroid_pixel(comp, w)
            result.append((cx, cy, area))
    return result


def connected_components_occupied(w: int, h: int, pixels: List[int]) -> List[List[int]]:
    """对占据像素（254/255）做连通分量。"""
    n = w * h
    visited = [False] * n
    components = []
    for start in range(n):
        if visited[start] or not _is_occupied(pixels[start]):
            continue
        comp = []
        q = deque([start])
        visited[start] = True
        while q:
            i = q.popleft()
            comp.append(i)
            for j in pixel_neighbors_4(w, h, i):
                if not visited[j] and _is_occupied(pixels[j]):
                    visited[j] = True
                    q.append(j)
        components.append(comp)
    return components


def _filter_interest_blob_by_bbox(
    w: int,
    h: int,
    pixels: List[int],
    comp: List[int],
    resolution: float,
    max_bbox_extent_m: float,
    white_ratio_min: float,
) -> Optional[Tuple[float, float, int]]:
    """
    白色中间的灰/黑连通块：包围盒 x、y 跨度（米）均 <= max_bbox_extent_m；
    不触边则接受；触边时需 boundary_white_ratio >= white_ratio_min。
    返回 (cx_px, cy_px, n_pixels) 供标注与排序；不按面积阈值过滤。
    """
    span_x_px, span_y_px = bbox_inclusive_spans_px(comp, w)
    if span_x_px * resolution > max_bbox_extent_m:
        return None
    if span_y_px * resolution > max_bbox_extent_m:
        return None
    cx, cy = centroid_pixel(comp, w)
    if not touches_border(w, h, comp):
        return (cx, cy, len(comp))
    if boundary_white_ratio(w, h, pixels, comp) >= white_ratio_min:
        return (cx, cy, len(comp))
    return None


def find_enclosed_blobs_in_map(
    w: int,
    h: int,
    pixels: List[int],
    resolution: float,
    max_bbox_extent_m: float = 0.30,
    white_ratio_min: float = 0.4,
    occupied_only: bool = False,
    dedupe_min_separation_px: float = 4.0,
) -> List[Tuple[float, float, int]]:
    """
    识别「白色中间」的小块：灰(205)与黑(254/255)分别做四连通，再合并候选列表，
    避免未知区与外墙在像素上连成一片时把整张图当成一块。
    黑灰四邻接的同一物体常会在两侧各形成一个分量，由 dedupe 合并为一点。
    仅按包围盒 x/y 跨度（米）过滤；整段贴外缘的条带由 component_lies_entirely_on_one_map_edge 剔除。
    occupied_only=True 时只对占据像素做连通域（不含 205）。
    """
    if occupied_only:
        all_blocks = connected_components_by_predicate(
            w, h, pixels, lambda v: _is_occupied(int(v))  # noqa: E731
        )
    else:
        all_blocks = connected_components(w, h, pixels, UNKNOWN) + connected_components_occupied(
            w, h, pixels
        )
    result: List[Tuple[float, float, int]] = []
    for comp in all_blocks:
        if component_lies_entirely_on_one_map_edge(comp, w, h):
            continue
        one = _filter_interest_blob_by_bbox(
            w, h, pixels, comp, resolution, max_bbox_extent_m, white_ratio_min
        )
        if one is not None:
            result.append(one)
    return dedupe_interest_blobs(result, dedupe_min_separation_px)


def get_interest_points_from_pgm(
    pgm_path: str,
    resolution: Optional[float] = None,
    origin: Optional[Tuple[float, float]] = None,
    max_bbox_extent_m: float = 0.30,
    white_ratio_min: float = 0.4,
    prefer_yaml: bool = True,
    occupied_only: bool = False,
    dedupe_min_separation_px: float = 4.0,
) -> List[Tuple[float, float]]:
    """
    从 PGM 地图中识别兴趣点，返回地图坐标系下的 (mx, my) 列表。
    供 task_manager 等节点调用，用于探索完成后基于地图的补检。
    """
    # 自动从同名 yaml 读取 resolution/origin（PGM 不包含原点信息）
    if prefer_yaml:
        meta = load_map_metadata_from_yaml(pgm_path)
        if meta is not None:
            resolution, origin = meta
    if resolution is None:
        resolution = DEFAULT_RESOLUTION
    if origin is None:
        origin = DEFAULT_ORIGIN

    w, h, pixels = load_pgm(pgm_path)
    blobs = find_enclosed_blobs_in_map(
        w,
        h,
        pixels,
        resolution,
        max_bbox_extent_m,
        white_ratio_min=white_ratio_min,
        occupied_only=occupied_only,
        dedupe_min_separation_px=dedupe_min_separation_px,
    )
    blobs.sort(key=lambda b: (-b[1], b[0]))
    points = []
    for cx_px, cy_px, area in blobs:
        mx, my = pixel_to_map(cx_px, cy_px, resolution, origin[0], origin[1], h)
        points.append((mx, my))
    return points


def run_detection_enclosed_blobs(
    pgm_path: str,
    resolution: float = DEFAULT_RESOLUTION,
    origin: Tuple[float, float] = DEFAULT_ORIGIN,
    max_bbox_extent_m: float = 0.30,
    occupied_only: bool = False,
    white_ratio_min: float = 0.4,
    dedupe_min_separation_px: float = 4.0,
    output_image: Optional[str] = None,
    no_plot: bool = False,
) -> None:
    """运行「白色中间有明显区块」识别，输出各区域中心坐标，并可选生成标注图。"""
    meta = load_map_metadata_from_yaml(pgm_path)
    if meta is not None:
        resolution, origin = meta
    w, h, pixels = load_pgm(pgm_path)
    print(f"地图尺寸: {w} x {h}, 分辨率: {resolution} m/px, 原点: {origin}")
    print(
        f"识别目标: 白底上灰/黑四连通块，包围盒 x/y 跨度均 <= {max_bbox_extent_m} m；"
        "触边时要求邻白比例阈值"
    )
    blobs = find_enclosed_blobs_in_map(
        w,
        h,
        pixels,
        resolution,
        max_bbox_extent_m,
        white_ratio_min=white_ratio_min,
        occupied_only=occupied_only,
        dedupe_min_separation_px=dedupe_min_separation_px,
    )
    # 按 y 降序、x 升序排列，便于与图中 3x3 从上到下、从左到右对应
    blobs.sort(key=lambda b: (-b[1], b[0]))
    print(f"识别到封闭区域数量: {len(blobs)}")
    print("-" * 60)
    for i, (cx_px, cy_px, area) in enumerate(blobs):
        mx, my = pixel_to_map(cx_px, cy_px, resolution, origin[0], origin[1], h)
        print(
            f"区域 {i+1}: "
            f"像素中心 ({cx_px:.1f}, {cy_px:.1f}), "
            f"地图坐标 ({mx:.3f}, {my:.3f}) m, "
            f"面积 {area} px"
        )
    if not no_plot and blobs:
        out = output_image
        if not out:
            out = str(Path(pgm_path).parent / (Path(pgm_path).stem + "_marked.png"))
        draw_result_image(w, h, pixels, blobs, out)


def run_detection(
    pgm_path: str,
    resolution: float = DEFAULT_RESOLUTION,
    origin: Tuple[float, float] = DEFAULT_ORIGIN,
    min_object_pixels: int = MIN_OBJECT_PIXELS,
    max_object_pixels: int = MAX_OBJECT_PIXELS,
    include_unknown_as_free: bool = False,
    output_image: Optional[str] = None,
    no_plot: bool = False,
) -> None:
    w, h, pixels = load_pgm(pgm_path)
    print(f"地图尺寸: {w} x {h}, 分辨率: {resolution} m/px, 原点: {origin}")
    if include_unknown_as_free:
        print("封闭空间定义: 可通行 = FREE(0) + UNKNOWN(205)")

    enclosed_regions = find_enclosed_free_regions(w, h, pixels, include_unknown=include_unknown_as_free)
    print(f"封闭空间数量: {len(enclosed_regions)}")

    all_objects: List[Tuple[float, float, float, float, int, int]] = []
    for ri, region in enumerate(enclosed_regions):
        objs = occupied_blobs_adjacent_to_free_region(
            w, h, pixels, set(region), min_object_pixels, max_object_pixels
        )
        for cx_px, cy_px, area in objs:
            mx, my = pixel_to_map(cx_px, cy_px, resolution, origin[0], origin[1], h)
            all_objects.append((mx, my, cx_px, cy_px, area, ri))

    print(f"封闭空间内物体数量: {len(all_objects)}")
    print("-" * 60)
    for i, (mx, my, cx_px, cy_px, area, region_id) in enumerate(all_objects):
        print(
            f"物体 {i+1} [封闭空间 {region_id}]: "
            f"像素中心 ({cx_px:.1f}, {cy_px:.1f}), "
            f"地图坐标 ({mx:.3f}, {my:.3f}) m, "
            f"面积 {area} px"
        )
    if not no_plot and all_objects:
        out = output_image or str(Path(pgm_path).parent / (Path(pgm_path).stem + "_marked.png"))
        points = [(cx, cy, area) for (_, _, cx, cy, area, _) in all_objects]
        draw_result_image(w, h, pixels, points, out)
    return


def main():
    parser = argparse.ArgumentParser(description="识别 PGM 地图封闭空间/封闭区域点位")
    default_pgm = Path(__file__).resolve().parent.parent / "maps" / "tb3_sandbox.pgm"
    parser.add_argument("pgm", nargs="?", default=str(default_pgm), help="PGM 地图路径")
    parser.add_argument(
        "--mode",
        "-m",
        choices=["rooms", "enclosed-blobs"],
        default="enclosed-blobs",
        help="rooms=可通行房间内物体; enclosed-blobs=白底上小灰黑块(包围盒米制) (默认: enclosed-blobs)",
    )
    parser.add_argument("--resolution", "-r", type=float, default=DEFAULT_RESOLUTION, help="地图分辨率 m/px")
    parser.add_argument(
        "--origin",
        type=float,
        nargs=2,
        default=list(DEFAULT_ORIGIN),
        metavar=("OX", "OY"),
        help="地图原点 (m)",
    )
    parser.add_argument(
        "--min-pixels",
        type=int,
        default=5,
        help="[rooms 模式] 封闭空间内占据物最小像素数",
    )
    parser.add_argument(
        "--max-pixels",
        type=int,
        default=500,
        help="[rooms 模式] 封闭空间内占据物最大像素数",
    )
    parser.add_argument(
        "--max-bbox-m",
        type=float,
        default=0.30,
        metavar="M",
        help="[enclosed-blobs] 包围盒 x、y 跨度(米)均不超过 M (默认 0.30)",
    )
    parser.add_argument(
        "--dedupe-px",
        type=float,
        default=4.0,
        metavar="P",
        help="[enclosed-blobs] 质心距离小于 P 像素时只保留面积较大的一块 (默认 4)",
    )
    parser.add_argument("--include-unknown", action="store_true", help="[rooms 模式] 将未知(205)也视为可通行")
    parser.add_argument(
        "--occupied-only",
        action="store_true",
        help="[enclosed-blobs] 仅按占据(254/255)识别；默认灰+黑(205+占据)四连通",
    )
    parser.add_argument(
        "--white-ratio",
        type=float,
        default=0.4,
        metavar="R",
        help="[enclosed-blobs] 接触边界时，边界邻接白色的比例>=R 才视为物体 (默认 0.4，放宽轮廓)",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        default=None,
        metavar="PATH",
        help="标注图输出路径（默认: 与 PGM 同目录，文件名为 xxx_marked.png）",
    )
    parser.add_argument("--no-plot", action="store_true", help="不生成标注图")
    args = parser.parse_args()
    if args.mode == "enclosed-blobs":
        run_detection_enclosed_blobs(
            args.pgm,
            resolution=args.resolution,
            origin=tuple(args.origin),
            max_bbox_extent_m=args.max_bbox_m,
            occupied_only=args.occupied_only,
            white_ratio_min=args.white_ratio,
            dedupe_min_separation_px=args.dedupe_px,
            output_image=args.output,
            no_plot=args.no_plot,
        )
    else:
        run_detection(
            args.pgm,
            resolution=args.resolution,
            origin=tuple(args.origin),
            min_object_pixels=args.min_pixels,
            max_object_pixels=args.max_pixels,
            include_unknown_as_free=args.include_unknown,
            output_image=args.output,
            no_plot=args.no_plot,
        )


if __name__ == "__main__":
    main()


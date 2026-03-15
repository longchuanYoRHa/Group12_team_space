#!/usr/bin/env python3
"""
PGM 地图封闭空间内物体点位识别样例程序（独立脚本，非 ROS 节点）

直接运行测试：
  cd real_Nav_test && python3 real_Nav_test/detect_objects_in_pgm_map.py
  python3 real_Nav_test/detect_objects_in_pgm_map.py maps/tb3_sandbox.pgm
  python3 real_Nav_test/detect_objects_in_pgm_map.py --mode enclosed-blobs --help

两种模式：
  default     最外侧封闭轮廓内的「可通行房间」→ 房间内占据物（盒子/台子）中心。
  enclosed-blobs  白色中间有明显区块即视为物体（不要求轮廓完全封闭）：按「边界邻接白色的比例」判定，建图不完整时更鲁棒。

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

# 地图语义（与 map_server 一致）
FREE = 0
# 占据：部分地图保存为 254 或 255，均视为障碍/物体
OCCUPIED_VALUES = (254, 255)
UNKNOWN = 205


def _is_occupied(pixel_val: int) -> bool:
    return pixel_val in OCCUPIED_VALUES

# 默认分辨率 (m/pixel)，与 mapper_params 一致；若使用 map.yaml 请从 yaml 读取
DEFAULT_RESOLUTION = 0.05
# 默认地图原点 (m)，若有 map.yaml 的 origin 请替换
DEFAULT_ORIGIN = (0.0, 0.0)

# 物体占位过滤：像素块面积范围（像素数），过滤噪声与过大的墙
MIN_OBJECT_PIXELS = 10
MAX_OBJECT_PIXELS = 2500


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


def connected_components_non_free(
    w: int, h: int, pixels: List[int]
) -> List[List[int]]:
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


def boundary_white_ratio(
    w: int, h: int, pixels: List[int], comp: List[int]
) -> float:
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
    px: float, py: float,
    resolution: float, origin_x: float, origin_y: float,
    height: int
) -> Tuple[float, float]:
    """像素坐标转地图坐标 (m)。图像 y 向下，地图 y 向上时需翻转。"""
    # 常见约定：图像 (0,0) 在左上，地图原点在左下，故 mx = ox + px*res, my = oy + (height-1-py)*res
    mx = origin_x + px * resolution
    my = origin_y + (height - 1 - py) * resolution
    return mx, my


def draw_result_image(
    w: int, h: int, pixels: List[int],
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
    w: int, h: int, pixels: List[int],
    free_region: set,
    min_pixels: int, max_pixels: int
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
    n = w * h
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


def _filter_one_component(
    w: int, h: int, pixels: List[int],
    comp: List[int],
    min_pixels: int, max_pixels: int,
    white_ratio_min: float,
) -> Optional[Tuple[float, float, int]]:
    """单个连通分量是否满足「物体」条件，满足则返回 (cx, cy, area)。"""
    area = len(comp)
    if area < min_pixels or area > max_pixels:
        return None
    if not touches_border(w, h, comp):
        cx, cy = centroid_pixel(comp, w)
        return (cx, cy, area)
    if boundary_white_ratio(w, h, pixels, comp) >= white_ratio_min:
        cx, cy = centroid_pixel(comp, w)
        return (cx, cy, area)
    return None


def find_enclosed_blobs_in_map(
    w: int, h: int, pixels: List[int],
    min_pixels: int = 5,
    max_pixels: int = 500,
    non_white: bool = True,
    white_ratio_min: float = 0.4,
) -> List[Tuple[float, float, int]]:
    """
    识别「白色中间有明显区块」的物体（不要求轮廓完全封闭，建图不完整时更鲁棒）。
    当 non_white=True 时，对 205(未知) 与 254/255(占据) 分别做连通分量再过滤，避免目标(小 205 块)
    与墙(254) 连成一片被过滤；单目标地图(如 my_map.pgm) 也能正确识别。
    """
    result = []
    if non_white:
        # 分别做 205 与 254/255 的连通分量，再统一过滤，避免小物体被大块合并
        blocks_205 = connected_components(w, h, pixels, UNKNOWN)
        blocks_occ = connected_components_occupied(w, h, pixels)
        all_blocks = blocks_205 + blocks_occ
    else:
        all_blocks = connected_components_occupied(w, h, pixels)
    for comp in all_blocks:
        one = _filter_one_component(
            w, h, pixels, comp, min_pixels, max_pixels, white_ratio_min
        )
        if one is not None:
            result.append(one)
    return result


def get_interest_points_from_pgm(
    pgm_path: str,
    resolution: float = DEFAULT_RESOLUTION,
    origin: Tuple[float, float] = DEFAULT_ORIGIN,
    min_pixels: int = 5,
    max_pixels: int = 500,
    non_white: bool = True,
    white_ratio_min: float = 0.4,
) -> List[Tuple[float, float]]:
    """
    从 PGM 地图中识别兴趣点，返回地图坐标系下的 (mx, my) 列表。
    供 task_manager 等节点调用，用于探索完成后基于地图的补检。
    """
    w, h, pixels = load_pgm(pgm_path)
    blobs = find_enclosed_blobs_in_map(
        w, h, pixels, min_pixels, max_pixels,
        non_white=non_white, white_ratio_min=white_ratio_min,
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
    min_pixels: int = 5,
    max_pixels: int = 500,
    non_white: bool = True,
    white_ratio_min: float = 0.4,
    output_image: Optional[str] = None,
    no_plot: bool = False,
) -> None:
    """运行「白色中间有明显区块」识别，输出各区域中心坐标，并可选生成标注图。"""
    w, h, pixels = load_pgm(pgm_path)
    print(f"地图尺寸: {w} x {h}, 分辨率: {resolution} m/px, 原点: {origin}")
    print("识别目标: 白色中间有明显区块即视为物体（轮廓不必完全封闭）")
    blobs = find_enclosed_blobs_in_map(
        w, h, pixels, min_pixels, max_pixels,
        non_white=non_white, white_ratio_min=white_ratio_min,
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
            w, h, pixels, set(region),
            min_object_pixels, max_object_pixels
        )
        for cx_px, cy_px, area in objs:
            mx, my = pixel_to_map(
                cx_px, cy_px,
                resolution, origin[0], origin[1], h
            )
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
    parser = argparse.ArgumentParser(
        description="识别 PGM 地图封闭空间/封闭区域点位"
    )
    default_pgm = Path(__file__).resolve().parent.parent / "maps" / "tb3_sandbox.pgm"
    parser.add_argument(
        "pgm",
        nargs="?",
        default=str(default_pgm),
        help="PGM 地图路径",
    )
    parser.add_argument(
        "--mode", "-m",
        choices=["rooms", "enclosed-blobs"],
        default="enclosed-blobs",
        help="rooms=可通行房间内物体; enclosed-blobs=最外侧轮廓内被白色包裹的封闭区域(如9个圆) (默认: enclosed-blobs)",
    )
    parser.add_argument(
        "--resolution", "-r",
        type=float,
        default=DEFAULT_RESOLUTION,
        help="地图分辨率 m/px",
    )
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
        help="封闭区域最小像素数 (enclosed-blobs 默认 5，以识别小圆孔)",
    )
    parser.add_argument(
        "--max-pixels",
        type=int,
        default=500,
        help="封闭区域最大像素数 (enclosed-blobs 默认 500，过滤外墙等大块)",
    )
    parser.add_argument(
        "--include-unknown",
        action="store_true",
        help="[rooms 模式] 将未知(205)也视为可通行",
    )
    parser.add_argument(
        "--occupied-only",
        action="store_true",
        help="[enclosed-blobs] 仅按占据(254/255)识别；默认按「非白色」识别",
    )
    parser.add_argument(
        "--white-ratio",
        type=float,
        default=0.4,
        metavar="R",
        help="[enclosed-blobs] 接触边界时，边界邻接白色的比例>=R 才视为物体 (默认 0.4，放宽轮廓)",
    )
    parser.add_argument(
        "--output", "-o",
        type=str,
        default=None,
        metavar="PATH",
        help="标注图输出路径（默认: 与 PGM 同目录，文件名为 xxx_marked.png）",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="不生成标注图",
    )
    args = parser.parse_args()
    if args.mode == "enclosed-blobs":
        run_detection_enclosed_blobs(
            args.pgm,
            resolution=args.resolution,
            origin=tuple(args.origin),
            min_pixels=args.min_pixels,
            max_pixels=args.max_pixels,
            non_white=not args.occupied_only,
            white_ratio_min=args.white_ratio,
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

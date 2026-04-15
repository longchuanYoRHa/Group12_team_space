from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Tuple

class TaskState(Enum):
    INIT = "init"
    PRE_EXPLORE_SCAN = "pre_explore_scan"
    EXPLORE = "explore"
    PRECISION_ALIGN = "precision_align"
    ARM_EXECUTE = "arm_execute"
    NAV_TO_INTEREST_POINT = "nav_to_interest_point"
    FINISHED = "finished"

class CargoState(Enum):
    EMPTY = "empty"
    HAS_OBJECT = "has_object"

@dataclass
class BoxInfo:
    color: str
    pose: Tuple[float, float]
    detected: bool = False

@dataclass
class Report:
    boxes: Dict[str, BoxInfo] = field(default_factory=dict)
    blocks_found: int = 0
    interest_points_total: int = 0
    interest_points_visited: int = 0
    interest_points_skipped: int = 0
    docking_attempts: int = 0
    docking_success: int = 0
    pick_place_success: int = 0

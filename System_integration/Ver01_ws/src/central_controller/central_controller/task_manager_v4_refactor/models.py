from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Union

import geometry_msgs.msg as geometry_msgs


class CargoState(Enum):
    EMPTY = "empty"
    HAS_OBJECT = "has_object"


class TaskState(Enum):
    INIT = "init"
    PRE_EXPLORE_SPIN = "pre_explore_spin"
    EXPLORE = "explore"
    PRECISION_ALIGN = "precision_align"
    GRASP = "grasp"
    RESUME_EXPLORE_FOR_BIN = "resume_explore_for_bin"
    NAV_TO_BIN_PREPLACE = "nav_to_bin_preplace"
    FORWARD_BEFORE_PLACE = "forward_before_place"
    PLACE_IN_BIN = "place_in_bin"
    BACKUP_AFTER_ACTION = "backup_after_action"
    POST_ACTION = "post_action"
    EXPLORE_FINISHED_FALLBACK = "explore_finished_fallback"
    RUN_MAP_DETECTION = "run_map_detection"
    NAV_TO_INTEREST_POINT = "nav_to_interest_point"


class NavPurpose(Enum):
    NONE = "none"
    PRE_EXPLORE_NAV = "pre_explore_nav"
    OBJECT_PREGRASP = "object_pregrasp"
    BIN_PREPLACE = "bin_preplace"
    INTEREST_POINT = "interest_point"
    BACKUP_AFTER_ACTION = "backup_after_action"


@dataclass(frozen=True)
class TickEvent:
    pass


@dataclass(frozen=True)
class ObjectVisionEvent:
    color: str
    point: geometry_msgs.Point


@dataclass(frozen=True)
class BinVisionEvent:
    color: str
    point: geometry_msgs.Point


@dataclass(frozen=True)
class ExploreFinishedEvent:
    pass


@dataclass(frozen=True)
class Nav2GoalResponseEvent:
    future: Any


@dataclass(frozen=True)
class Nav2ResultEvent:
    future: Any


TaskEvent = Union[
    TickEvent,
    ObjectVisionEvent,
    BinVisionEvent,
    ExploreFinishedEvent,
    Nav2GoalResponseEvent,
    Nav2ResultEvent,
]


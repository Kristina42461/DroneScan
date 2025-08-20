
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
import math

@dataclass(frozen=True)
class Point:
    x: float
    y: float
    z: float = 0.0

@dataclass
class Waypoint:
    p: Point
    hold_sec: float = 0.0
    speed_mps: float = 3.0

@dataclass(frozen=True)
class Task:
    id: str
    priority: float           # p_t ∈ [0,1]
    target: Point             # центр ячейки/сегмента
    aoi_id: Optional[str] = None

@dataclass
class DroneState:
    drone_id: str
    pos: Point
    battery_rem_Wh: float
    speed_cruise_mps: float = 3.0

Plan = List[Waypoint]
Assignment = Dict[str, List[Task]]  # {drone_id: [Task,...]}


from dataclasses import dataclass
from typing import Optional

@dataclass
class Obstacle:
    # Положение и (опционально) скорость препятствия в ЛСК дрона, метры/мс
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    radius: float = 0.4  # эффективный радиус "зоны" препятствия

@dataclass
class AvoidanceInput:
    # Входы на такте управления
    ref_vx: float           # желаемая скорость по маршруту (м/с)
    ref_vy: float
    dt: float               # шаг дискретизации, с
    progress_ds: float      # прирост прогресса вдоль пути, м (за dt)
    front_blocked_ratio: float  # 0..1 — доля занятости во фронтальном секторе
    v_prev_x: float         # предыдущая выданная скорость (для сглаживания)
    v_prev_y: float
    vmax: float = 4.0
    obstacles: list[Obstacle] = None  # список препятствий

@dataclass
class AvoidanceOutput:
    vx: float
    vy: float
    in_recovery: bool = False
    waypoint_override_x: Optional[float] = None
    waypoint_override_y: Optional[float] = None

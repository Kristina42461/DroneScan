
from typing import Dict, List, Tuple
from schemas import Task, Waypoint, Point, Assignment, DroneState
import math

class HybridCoverage:
    """
    Генерация 'бустрофедонных' полос с адаптивным шагом d(x,y) и простым
    локальным добором. Без внешних геометрических библиотек — упрощённо:
    работаем по ограничивающему прямоугольнику каждой задачи/ячейки.
    """
    def __init__(self, fov_m: float=20.0, overlap_perp: float=0.3, kappa: float=0.7,
                 cruise_speed_mps: float=3.0, altitude_m: float=20.0):
        self.fov_m = fov_m
        self.overlap_perp = overlap_perp
        self.kappa = kappa
        self.cruise = cruise_speed_mps
        self.alt = altitude_m

    def _base_step(self) -> float:
        return max(1.0, self.fov_m * (1.0 - self.overlap_perp))

    def _adaptive_step(self, priority: float) -> float:
        # d(x,y) = d0 / (1 + κ * w), w ∈ [0,1]
        d0 = self._base_step()
        return d0 / (1.0 + self.kappa * max(0.0, min(1.0, priority)))

    def _stripe_route(self, center: Point, size: float, step: float, along_x: bool=True) -> List[Waypoint]:
        """
        Строим набор параллельных проходов в квадратной 'ячейке' вокруг center
        со стороной 'size'. Это упрощённая иллюстрация без вычитания препятствий.
        """
        half = size / 2.0
        waypoints: List[Waypoint] = []
        # число полос
        n = max(1, int(math.ceil(size / step)))
        for i in range(n):
            offset = -half + i * step
            if along_x:
                # линии параллельны оси X, двигаемся по Y
                y = center.y + offset
                x1, x2 = center.x - half, center.x + half
                row = [Point(x1, y, self.alt), Point(x2, y, self.alt)]
            else:
                x = center.x + offset
                y1, y2 = center.y - half, center.y + half
                row = [Point(x, y1, self.alt), Point(x, y2, self.alt)]
            # чередуем направление, чтобы уменьшить холостые пробеги
            pts = row if i % 2 == 0 else list(reversed(row))
            waypoints += [Waypoint(p, speed_mps=self.cruise) for p in pts]
        return waypoints

    def build_plans(self, assignment: Assignment, drones: List[DroneState],
                    cell_size_m: float=60.0, along_x: bool=True) -> Dict[str, List[Waypoint]]:
        """
        На вход: распределение задач {drone_id: [Task]}.
        На выход: {drone_id: [Waypoint,...]} для выполнения покрытия по каждой задаче.
        """
        plans: Dict[str, List[Waypoint]] = {d.drone_id: [] for d in drones}
        for drone_id, tasks in assignment.items():
            seq: List[Waypoint] = []
            # грубая 'сшивка': сортировка задач по расстоянию (жадная эвристика)
            cur = next(d for d in drones if d.drone_id == drone_id).pos
            remaining = tasks[:]
            while remaining:
                # ближайшая следующая задача
                nxt = min(remaining, key=lambda t: (cur.x - t.target.x)**2 + (cur.y - t.target.y)**2)
                remaining.remove(nxt)
                step = self._adaptive_step(nxt.priority)
                route = self._stripe_route(center=nxt.target, size=cell_size_m, step=step, along_x=along_x)
                seq.extend(route)
                if route:
                    cur = route[-1].p
            plans[drone_id] = seq
        return plans

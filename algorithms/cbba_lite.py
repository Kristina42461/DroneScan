
from typing import Dict, List, Tuple
from schemas import Task, DroneState
import math
import random

class CBBALite:
    """
    Упрощённый аукцион CBBA-lite: каждый агент ставит 'ставку' за задачу
    на основе маржинальной полезности от лучшей вставки в текущий маршрут.
    Консенсус достигается за 3–6 раундов обмена (в рамках одного процесса
    моделируем синхронными раундами).
    """
    def __init__(self, alpha: float=1.0, beta: float=0.02, gamma: float=0.08,
                 rounds: int=5, max_tasks_per_agent: int=999):
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.rounds = rounds
        self.max_tasks_per_agent = max_tasks_per_agent

    def _dist(self, a, b) -> float:
        return math.hypot(a.x-b.x, a.y-b.y)

    def _marginal_gain(self, drone: DroneState, route, t: Task) -> Tuple[float, int]:
        """
        Оценка u_{i,t} = α*p_t - β*ΔT - γ*перерасход энергии.
        Вставляем t в лучшую позицию в route; возвращаем (gain, idx).
        """
        if len(route) == 0:
            travel = self._dist(drone.pos, t.target)
            delta_time = travel / max(drone.speed_cruise_mps, 0.1)
            energy_over = max(0.0, 0.0)  # без точной модели — 0, расширяется в energy-модуле
            gain = self.alpha*t.priority - self.beta*delta_time - self.gamma*energy_over
            return gain, 0

        best_gain, best_idx = -1e9, 0
        for i in range(len(route)+1):
            # простая оценка вставки между соседями
            prev_p = drone.pos if i == 0 else route[i-1].target
            next_p = route[i].target if i < len(route) else route[-1].target
            added = self._dist(prev_p, t.target) + self._dist(t.target, next_p)
            removed = self._dist(prev_p, next_p)
            delta = max(0.0, added - removed)
            delta_time = delta / max(drone.speed_cruise_mps, 0.1)
            energy_over = 0.0
            gain = self.alpha*t.priority - self.beta*delta_time - self.gamma*energy_over
            if gain > best_gain:
                best_gain, best_idx = gain, i
        return best_gain, best_idx

    def assign(self, tasks: List[Task], drones: List[DroneState]) -> Dict[str, List[Task]]:
        """
        Возвращает распределение задач {drone_id: [Task,...]}.
        Итеративно разрешает конфликты ставок; tie-break по drone_id.
        """
        routes: Dict[str, List[Task]] = {d.drone_id: [] for d in drones}
        claim: Dict[str, Tuple[str, float]] = {}  # task_id -> (winner_id, bid)

        # кандидаты задач
        pool = list(tasks)

        for _ in range(self.rounds):
            bids: Dict[str, Tuple[str, float, int]] = {}  # task_id -> (drone_id, bid, idx)
            # локальные ставки
            for d in drones:
                if len(routes[d.drone_id]) >= self.max_tasks_per_agent:
                    continue
                for t in pool:
                    gain, idx = self._marginal_gain(d, routes[d.drone_id], t)
                    if t.id not in bids or gain > bids[t.id][1] or \
                       (abs(gain - bids[t.id][1]) < 1e-9 and d.drone_id < bids[t.id][0]):
                        bids[t.id] = (d.drone_id, gain, idx)

            # согласование и фиксация лучших ставок
            changed = False
            for t in pool[:]:
                if t.id not in bids:
                    continue
                winner, bid, idx = bids[t.id]
                prev = claim.get(t.id)
                if (prev is None) or (bid > prev[1]) or (abs(bid-prev[1])<1e-9 and winner < prev[0]):
                    claim[t.id] = (winner, bid)
                    # вставка в маршрут победителя
                    r = routes[winner]
                    r.insert(min(idx, len(r)), t)
                    routes[winner] = r
                    pool.remove(t)
                    changed = True
            if not changed:
                break

        return routes

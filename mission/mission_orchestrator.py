
from typing import List, Dict
from schemas import Task, DroneState, Waypoint
from cbba_lite import CBBALite
from coverage_hybrid import HybridCoverage

class MissionOrchestrator:
    """
    Лёгкий оркестратор для этапа распределения и генерации маршрутов покрытия.
    Реализует только ту часть цикла, которая нужна в этой главе: CBBA + Coverage.
    """
    def __init__(self, drones: List[DroneState]):
        self.drones = drones
        self.cbba = CBBALite(alpha=1.0, beta=0.03, gamma=0.1, rounds=5)
        self.coverage = HybridCoverage(fov_m=22.0, overlap_perp=0.25, kappa=0.6,
                                       cruise_speed_mps=3.0, altitude_m=22.0)

    def plan_from_tasks(self, tasks: List[Task]) -> Dict[str, List[Waypoint]]:
        """
        1) распределяет задачи между дронами,
        2) строит покрытие для каждой закреплённой задачи,
        3) возвращает набор путевых точек на каждого дрона.
        """
        assignment = self.cbba.assign(tasks, self.drones)
        plans = self.coverage.build_plans(assignment, self.drones,
                                          cell_size_m=60.0, along_x=True)
        return plans

# --- Пример использования (встраивание без изменения существующих модулей) ---

def build_mission_plans(drone_states: List[DroneState], task_list: List[Task]) -> Dict[str, List[Waypoint]]:
    """
    Функция-обёртка: может вызываться из существующей логики запуска миссии.
    На вход: актуальная телеметрия дронов и список задач (ячейки покрытия).
    На выход: готовые waypoint-последовательности для каждого дрона.
    """
    orch = MissionOrchestrator(drone_states)
    return orch.plan_from_tasks(task_list)

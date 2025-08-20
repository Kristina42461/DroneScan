
from typing import Dict, List
from schemas import Task, Waypoint, DroneState, Point
from mission_orchestrator import build_mission_plans
from cbba_lite import CBBALite
from coverage_hybrid import HybridCoverage
from avoidance_bindings import AvoidanceManager
from energy_rtb import EnergyRTBManager, LinkStats, RTBDecision

class MissionRunner:
    """
    Высокоуровневый цикл выполнения миссии:
    - формирование планов (CBBA + Coverage),
    - локальная безопасность (Avoidance),
    - контроль энергии/связи (EnergyRTB),
    - перераспределение задач и RTB по мере необходимости.
    """
    def __init__(self, drones: List[DroneState], home: Point, lz_list: List[Point]):
        self.drones = {d.drone_id: d for d in drones}
        self.home = home
        self.cbba = CBBALite()
        self.coverage = HybridCoverage()
        self.avoid = AvoidanceManager(drone_ids=list(self.drones.keys()))
        self.energy = EnergyRTBManager(home=home, lz_list=lz_list, altitude_m=22.0)
        self.active_plans: Dict[str, List[Waypoint]] = {d: [] for d in self.drones.keys()}
        self.finished: Dict[str, bool] = {d: False for d in self.drones.keys()}

    # --- внешние зависимости: должны предоставляться существующей системой ---
    def _telemetry(self, drone_id: str):
        """
        Заглушка: существующая система должна предоставить
        - battery_rem_Wh
        - link stats (rssi/snr/loss)
        - remaining distance of plan
        - vmax, прогресс, front_blocked_ratio, препятствия
        - текущую позицию (обновление self.drones[drone_id].pos)
        """
        raise NotImplementedError

    def _flight_cmd_set_velocity(self, drone_id: str, vx: float, vy: float, vz: float=0.0):
        """Выдаёт команду скорости в существующий полётный контроллер."""
        raise NotImplementedError

    def _flight_cmd_execute_waypoint(self, drone_id: str, wp: Waypoint):
        """Опционально: отправка WP в существующий исполнитель."""
        pass

    def prepare(self, tasks: List[Task]):
        # 1) начальное распределение и построение маршрутов
        drone_list = list(self.drones.values())
        plans = build_mission_plans(drone_list, tasks)  # CBBA + Coverage
        self.active_plans.update(plans)

    def _rebalance_leftover(self, giver_id: str, tasks_left: List[Task]):
        """
        При уходе дрона в RTB или посадку: возвращаем его невыполненные задачи в пул и
        перераспределяем среди оставшихся (CBBA-lite).
        """
        alive = [self.drones[k] for k, v in self.finished.items() if not v and k != giver_id]
        if not alive or not tasks_left:
            return
        assignment = self.cbba.assign(tasks_left, alive)
        # обновляем планы для получателей
        for rid, tlist in assignment.items():
            add_plan = self.coverage.build_plans({rid: tlist}, alive, cell_size_m=60.0)[rid]
            self.active_plans[rid].extend(add_plan)

    def step(self, dt: float = 0.1):
        """
        Один такт выполнения: на каждом дроне
        - контролируем избегание препятствий,
        - проверяем энергетику/связь,
        - при необходимости — RTB/посадка и перераспределение.
        """
        for drone_id, dstate in self.drones.items():
            if self.finished[drone_id]:
                continue

            # нет планов — значит дрон закончил свою часть
            if not self.active_plans[drone_id]:
                self.finished[drone_id] = True
                continue

            # референсное движение к следующей точке
            next_wp = self.active_plans[drone_id][0]

            # --- получить телеметрию из существующей системы ---
            tel = self._telemetry(drone_id)
            # ожидаем поля:
            # tel.battery_rem_Wh, tel.link (rssi/snr/loss), tel.remaining_m,
            # tel.vmax, tel.progress_ds, tel.front_blocked_ratio, tel.obstacles

            # --- локальная безопасность (Reactive Avoidance) ---
            # примитивная проекция "референса" в скорость по XY:
            ref_vx = (next_wp.p.x - dstate.pos.x)
            ref_vy = (next_wp.p.y - dstate.pos.y)
            # нормируем к vmax
            import math
            norm = max(1e-3, math.hypot(ref_vx, ref_vy))
            ref_vx = ref_vx / norm * min(tel.vmax, next_wp.speed_mps)
            ref_vy = ref_vy / norm * min(tel.vmax, next_wp.speed_mps)

            out = self.avoid.compute(drone_id, ref_vx, ref_vy, dt=dt,
                                     progress_ds=tel.progress_ds,
                                     front_blocked_ratio=tel.front_blocked_ratio,
                                     obstacles=tel.obstacles, vmax=tel.vmax)
            self._flight_cmd_set_velocity(drone_id, out.vx, out.vy, 0.0)

            # --- проверка достижения WP ---
            reached = (math.hypot(next_wp.p.x - dstate.pos.x, next_wp.p.y - dstate.pos.y) < 1.0)
            if reached:
                self.active_plans[drone_id].pop(0)

            # --- энергетика/связь и решение RTB ---
            link = LinkStats(rssi=tel.link.rssi, snr=tel.link.snr, loss_rate=tel.link.loss_rate)
            decision: RTBDecision = self.energy.decide(
                drone=dstate,
                battery_rem_Wh=tel.battery_rem_Wh,
                link=link,
                remaining_plan_m=tel.remaining_m
            )

            if decision.mode == "continue":
                continue
            elif decision.mode == "simplify":
                # упрощаем план: увеличим шаг покрытия → оставшиеся WP прореживаем через один
                if len(self.active_plans[drone_id]) > 2:
                    self.active_plans[drone_id] = self.active_plans[drone_id][::2]
            elif decision.mode in ("rtb", "land_lz"):
                # собираем невыполненные задачи дрона (для примера: группируем по центрам WP)
                # в реальной системе здесь у нас есть список Task; используем приблизительную реконструкцию
                leftover_tasks: List[Task] = []  # заготовка под интеграцию с хранилищем задач
                # заменяем текущий план на RTB/LZ
                self.active_plans[drone_id] = decision.rtb_plan or []
                # инициируем перераспределение
                self._rebalance_leftover(giver_id=drone_id, tasks_left=leftover_tasks)
                # если путь пуст или прибыл домой — считаем завершённым
                if not self.active_plans[drone_id]:
                    self.finished[drone_id] = True

    def all_finished(self) -> bool:
        return all(self.finished.values())

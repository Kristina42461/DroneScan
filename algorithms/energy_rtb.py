
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
from schemas import Point, Waypoint, DroneState

@dataclass
class LinkStats:
    rssi: float            # dBm (или нормированная метрика 0..1)
    snr: float             # dB
    loss_rate: float       # 0..1

@dataclass
class EnergyModel:
    p_cruise_W: float = 65.0      # мощность в крейсерском полёте
    p_hover_W: float  = 70.0      # мощность в висении
    p_maneuver_W: float = 80.0    # на манёврах/разворотах
    v_cruise_mps: float = 3.0

@dataclass
class EnergyPolicy:
    reserve_frac: float = 0.20      # обязательный резерв ёмкости
    ok_margin: float = 1.5           # ΔE_ok = 1.5× от ĤE_RTB
    min_margin: float = 1.1          # ΔE_min = 1.1× от ĤE_RTB
    q_ok: float = 0.7                # допустимое качество связи
    q_crit: float = 0.4              # критическое качество связи

@dataclass
class RTBDecision:
    mode: str                        # "continue" | "simplify" | "handoff" | "rtb" | "land_lz"
    rtb_plan: Optional[List[Waypoint]] = None
    target_lz: Optional[Point] = None

def _link_quality(ls: LinkStats) -> float:
    # нормированная оценка канала (пример): смешиваем RSSI/SNR/потери
    # приводим к 0..1 (грубая модель для иллюстрации)
    rssi_q = max(0.0, min(1.0, (ls.rssi + 90.0) / 40.0))  # -90..-50 → 0..1
    snr_q  = max(0.0, min(1.0, ls.snr / 30.0))            # 0..30 → 0..1
    loss_q = 1.0 - max(0.0, min(1.0, ls.loss_rate))       # 0 потерь → 1.0
    return 0.5*rssi_q + 0.3*snr_q + 0.2*loss_q

def _dist(a: Point, b: Point) -> float:
    import math
    return math.hypot(a.x - b.x, a.y - b.y)

def _straight_path(a: Point, b: Point, altitude: float, step_m: float = 30.0) -> List[Waypoint]:
    # простая ломаная (прямая) до базы/ЛЗ
    import math
    dx, dy = b.x - a.x, b.y - a.y
    L = math.hypot(dx, dy)
    n = max(1, int(L // step_m))
    wps = []
    for i in range(1, n+1):
        t = i / n
        wps.append(Waypoint(Point(a.x + t*dx, a.y + t*dy, altitude), speed_mps=3.0))
    return wps

class EnergyRTBManager:
    """
    Онлайн-контроль энергобаланса и качества канала, выдаёт решение по каждому дрону:
    продолжать / упростить план / передать задачи / RTB / посадка на ближайшую LZ.
    """
    def __init__(self, home: Point, lz_list: List[Point], altitude_m: float = 22.0,
                 model: EnergyModel = EnergyModel(), policy: EnergyPolicy = EnergyPolicy()):
        self.home = home
        self.lz_list = lz_list
        self.alt = altitude_m
        self.model = model
        self.policy = policy

    def _estimate_rtb_energy(self, cur: Point) -> float:
        D = _dist(cur, self.home)
        t = D / max(self.model.v_cruise_mps, 0.1)
        return self.model.p_cruise_W * t + self._reserve_energy(t)

    def _reserve_energy(self, t_sec: float) -> float:
        # Резерв: фиксированный минимум + манёвры
        return self.model.p_hover_W * 120.0 + self.model.p_maneuver_W * 10.0

    def _pick_lz(self, cur: Point) -> Point:
        # выбираем ближайшую LZ
        return min(self.lz_list, key=lambda p: _dist(cur, p)) if self.lz_list else self.home

    def decide(self,
               drone: DroneState,
               battery_rem_Wh: float,
               link: LinkStats,
               remaining_plan_m: float) -> RTBDecision:
        """
        remaining_plan_m — оценка оставшейся длины маршрута (м).
        Возвращает решение для данного дрона на текущем такте.
        """
        q = _link_quality(link)
        # энергозатраты на остаток плана (крейсер)
        t_plan = remaining_plan_m / max(self.model.v_cruise_mps, 0.1)
        E_mission = self.model.p_cruise_W * t_plan + self.model.p_maneuver_W * 5.0
        E_rtb = self._estimate_rtb_energy(drone.pos)

        # "резерв по безопасности" — сколько останется после миссии и RTB
        delta_E = battery_rem_Wh - (E_mission + E_rtb)

        # пороги
        ok = (delta_E >= self.policy.ok_margin * E_rtb) and (q >= self.policy.q_ok)
        yellow = (delta_E >= self.policy.min_margin * E_rtb) and (q >= self.policy.q_crit)

        if ok:
            return RTBDecision(mode="continue")
        if yellow:
            return RTBDecision(mode="simplify")  # упростить план (увеличить шаг полос, пропустить low-prio)
        # красная зона
        # проверяем достижимость базы, иначе — LZ
        if battery_rem_Wh >= E_rtb:
            rtb_path = _straight_path(drone.pos, self.home, altitude=self.alt)
            return RTBDecision(mode="rtb", rtb_plan=rtb_path)
        else:
            lz = self._pick_lz(drone.pos)
            lz_path = _straight_path(drone.pos, lz, altitude=self.alt)
            return RTBDecision(mode="land_lz", rtb_plan=lz_path, target_lz=lz)

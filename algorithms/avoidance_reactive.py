
import math
from typing import List, Tuple
from avoidance_types import Obstacle, AvoidanceInput, AvoidanceOutput

def _dot(ax, ay, bx, by): return ax*bx + ay*by
def _norm(ax, ay): return math.hypot(ax, ay)
def _angle(ax, ay): return math.atan2(ay, ax)
def _wrap(a): 
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

class ReactiveAvoidanceAgent:
    """
    Реализует:
    1) Формирование множества допустимых скоростей по TTC/клиренсу.
    2) Оптимизацию J(v) с приоритетом на следование ref_v.
    3) Детектор тупика (Dead-Wall) и режим восстановления Look-and-Turn.
    """
    def __init__(self,
                 tau_min: float = 1.8,      # минимально допустимый TTC, с
                 clr_min: float = 0.8,      # минимальный клиренс, м
                 w_goal: float = 1.0,
                 w_clr: float = 0.8,
                 w_ttc: float = 0.6,
                 w_sm: float = 0.3,
                 dead_progress_eps: float = 0.4,  # м за окно
                 dead_window_sec: float = 2.0,
                 front_block_thr: float = 0.65,
                 scan_step_deg: float = 15.0):
        self.tau_min = tau_min
        self.clr_min = clr_min
        self.w_goal = w_goal
        self.w_clr = w_clr
        self.w_ttc = w_ttc
        self.w_sm = w_sm

        self.dead_eps = dead_progress_eps
        self.dead_T = dead_window_sec
        self.front_thr = front_block_thr

        self.scan_step = math.radians(scan_step_deg)
        self.recovery = False
        self.recovery_phase = 0    # 0..N
        self.recovery_sign = 1     # чередование сторон
        self.acc_time = 0.0        # накопленное время в окне
        self.acc_progress = 0.0

    # --- базовые оценки безопасности ---

    def _ttc_of(self, vx: float, vy: float, o: Obstacle) -> float:
        # TTC при относительном движении (классическая формула для круговых объектов)
        rx, ry = o.x, o.y
        rvx, rvy = vx - o.vx, vy - o.vy
        v2 = rvx*rvx + rvy*rvy
        if v2 < 1e-6: 
            return float('inf')
        c = _dot(rx, ry, rvx, rvy)
        if c >= 0.0:  # удаляемся
            return float('inf')
        d2 = rx*rx + ry*ry
        r_safe = max(o.radius, 0.1)
        disc = c*c - v2*(d2 - r_safe*r_safe)
        if disc < 0.0:
            return float('inf')
        t1 = (-c - math.sqrt(max(0.0, disc))) / v2
        return t1 if t1 >= 0 else float('inf')

    def _min_ttc(self, vx: float, vy: float, obs: List[Obstacle]) -> float:
        if not obs: return float('inf')
        return min(self._ttc_of(vx, vy, o) for o in obs)

    def _clearance_along(self, vx: float, vy: float, obs: List[Obstacle]) -> float:
        # приближённый клиренс: минимальная ортогональная дистанция луча скорости до центров препятствий
        if not obs: return float('inf')
        vnorm = _norm(vx, vy) + 1e-6
        ux, uy = vx / vnorm, vy / vnorm
        best = float('inf')
        for o in obs:
            # расстояние от точки (0,0) до прямой along v, проходящей через (0,0), к точке (o.x, o.y)
            # |(o × u)| = |o.x*uy - o.y*ux|
            dist = abs(o.x*uy - o.y*ux) - o.radius
            best = min(best, dist)
        return best

    # --- генерация кандидатов и выбор решения ---

    def _candidates(self, vref_x: float, vref_y: float, vmax: float) -> List[Tuple[float,float]]:
        ang0 = _angle(vref_x, vref_y)
        vref = _norm(vref_x, vref_y)
        speeds = [max(0.5, min(vref, vmax)), min(vmax, max(1.0, vref+0.5))]
        angles = [ang0 + k*self.scan_step for k in range(-4, 5)]
        cand = []
        for a in angles:
            for s in speeds:
                cand.append((s*math.cos(a), s*math.sin(a)))
        # добавим торможение/стоп на случай полного перекрытия
        cand.append((0.0, 0.0))
        return cand

    def _pick(self, ain: AvoidanceInput) -> Tuple[float, float, float, float]:
        cand = self._candidates(ain.ref_vx, ain.ref_vy, ain.vmax)
        ang_ref = _angle(ain.ref_vx, ain.ref_vy)
        bestJ, best = 1e9, (0.0, 0.0, float('inf'), float('inf'))

        for vx, vy in cand:
            ttc = self._min_ttc(vx, vy, ain.obstacles or [])
            clr = self._clearance_along(vx, vy, ain.obstacles or [])
            # жёсткие пороги безопасности
            if ttc < self.tau_min or clr < self.clr_min:
                continue
            # стоимость
            dang = abs(_wrap(_angle(vx, vy) - ang_ref))
            J = (self.w_goal*dang 
                + self.w_clr*(0.7/ (clr + 1e-3))
                + self.w_ttc*(0.7/ (ttc + 1e-3))
                + self.w_sm*_norm(vx-ain.v_prev_x, vy-ain.v_prev_y))
            if J < bestJ:
                bestJ, best = J, (vx, vy, ttc, clr)

        # если не нашли безопасных кандидатов — тормозим и сохраняем ориентацию
        if bestJ >= 1e8:
            return 0.0, 0.0, 0.0, 0.0
        return best

    # --- детектор тупика и recovery ---

    def _update_deadwall(self, ain: AvoidanceInput):
        self.acc_time += ain.dt
        self.acc_progress += max(0.0, ain.progress_ds)
        if self.acc_time >= self.dead_T:
            dead = (self.acc_progress < self.dead_eps) and (ain.front_blocked_ratio >= self.front_thr)
            # сброс окна
            self.acc_time = 0.0
            self.acc_progress = 0.0
            if dead and not self.recovery:
                self.recovery = True
                self.recovery_phase = 1
                self.recovery_sign *= -1  # чередуем сторону
            elif (not dead) and self.recovery:
                # прогресс есть — выходим из recovery
                self.recovery = False
                self.recovery_phase = 0

    def _recovery_step(self, ain: AvoidanceInput) -> Tuple[float, float, float, float]:
        # стратегия Look-and-Turn: увеличиваем амплитуду сканирования
        vmag = max(0.6, min(ain.vmax, 1.2))
        ang_ref = _angle(ain.ref_vx, ain.ref_vy)
        amp = self.recovery_phase * self.scan_step
        a = ang_ref + self.recovery_sign * amp
        vx, vy = vmag*math.cos(a), vmag*math.sin(a)
        self.recovery_phase = min(self.recovery_phase + 1, 6)
        return vx, vy, float('inf'), float('inf')

    # --- основной такт ---

    def step(self, ain: AvoidanceInput) -> AvoidanceOutput:
        self._update_deadwall(ain)

        if self.recovery:
            vx, vy, _, _ = self._recovery_step(ain)
            return AvoidanceOutput(vx=vx, vy=vy, in_recovery=True)

        vx, vy, ttc, clr = self._pick(ain)
        # если всё перекрыто — включаем recovery
        if vx == 0.0 and vy == 0.0 and ain.front_blocked_ratio >= self.front_thr:
            self.recovery = True
            self.recovery_phase = 1
            self.recovery_sign *= -1
            vx, vy, _, _ = self._recovery_step(ain)
            return AvoidanceOutput(vx=vx, vy=vy, in_recovery=True)

        return AvoidanceOutput(vx=vx, vy=vy, in_recovery=False)

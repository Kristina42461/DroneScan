
from typing import List, Tuple
from avoidance_types import Obstacle

def obstacles_from_detections(dets: List[Tuple[float,float,float,float]],
                              fx: float, fy: float, z: float,
                              px_to_m: float = 0.002) -> List[Obstacle]:
    """
    Простейшее приближение: bbox → угловое смещение → локальная проекция на плоскость на высоте z.
    dets: [(cx_px, cy_px, w_px, h_px), ...]
    fx, fy: фокусные в пикселях; z: высота, м.
    px_to_m — эмпирический коэффициент пересчёта.
    """
    obs: List[Obstacle] = []
    for (cx, cy, w, h) in dets:
        # отклонение луча по центру бокса
        ax = (cx / fx)
        ay = (cy / fy)
        # локальная "глубина" через эмпирический масштаб
        R = max(w, h) * px_to_m
        # позиция препятствия в лобовом секторе (очень грубо)
        x = (1.0 / (abs(ax) + 1e-3)) * px_to_m * (1 if ax >= 0 else -1)
        y = (1.0 / (abs(ay) + 1e-3)) * px_to_m
        obs.append(Obstacle(x=x, y=y, radius=R))
    return obs


from schemas import Task, DroneState, Point
from mission_runner import MissionRunner

# Задаём парк и базу
drones = [
    DroneState(drone_id="dr1", pos=Point(0,0,0), battery_rem_Wh=60.0, speed_cruise_mps=3.0),
    DroneState(drone_id="dr2", pos=Point(10,0,0), battery_rem_Wh=65.0, speed_cruise_mps=3.0),
]
home = Point(0, -50, 0)
lz_list = [Point(30, -10, 0), Point(-25, -20, 0)]

# Пример задач покрытия (центры ячеек, приоритеты)
tasks = [
    Task(id="A1", priority=0.9, target=Point(40, 40, 0), aoi_id="A"),
    Task(id="A2", priority=0.7, target=Point(90, 40, 0), aoi_id="A"),
    Task(id="A3", priority=0.5, target=Point(40, 90, 0), aoi_id="A"),
    Task(id="A4", priority=0.6, target=Point(90, 90, 0), aoi_id="A"),
]

runner = MissionRunner(drones=drones, home=home, lz_list=lz_list)
runner.prepare(tasks)

# Далее цикл step(dt) должен вызываться существующей системой с предоставлением телеметрии
# (здесь опущено, т.к. интеграция идёт без изменения ваших исходников).

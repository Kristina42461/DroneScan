# DroneScan
🚀 Назначение
Система управления роем дронов с голосовым интерфейсом: «голос → сценарий → план миссии → полёт». Включает:
    распределение задач между БПЛА (CBBA-lite),
    адаптивное покрытие зоны (Hybrid Frontier–Lawnmower),
    реактивный обход препятствий (Reactive VO + Dead-Wall),
    энерго-адаптивное репланирование и безопасный RTB/LZ.
🧩 Требования
    Python 3.10+ (рекомендовано 3.10.x)
    OS: Windows 10/11 (для PyCaw/команд), Linux поддерживается без PyCaw-зависимостей
    БПЛА: DJI Tello / Tello EDU (управление через djitellopy)
    Микрофон/динамики для голосового интерфейса
    GPU не обязателен (Silero TTS может работать на CPU)
📦 Установка
 # 1) Клонирование
  git clone <repo-url> && cd <repo>

 # 2) Виртуальная среда
  python -m venv .venv
  . .venv/Scripts/activate    # Win
  # source .venv/bin/activate # Linux/Mac

 # 3) Зависимости
  pip install --upgrade pip
  pip install -r requirements.txt
  
  Модели и ресурсы
    1. Vosk (ASR)
    Скачайте русскую модель и распакуйте в assets/models/vosk/ru/.
    Укажите путь в config.py (см. шаблон ниже).
    2. Porcupine (Wake Word)
    Поместите файлы .ppn и параметры в assets/models/porcupine/.
    Задайте ключ доступа и путь к keyword в config.py.
    3. Звуки ассистента
    Файлы коротких сигналов в assets/sounds/ (greet.wav, ok.wav, ready.wav, off.wav, …).
    4. OpenAI (опционально)
    Для gpt_integration.py задайте OPENAI_API_KEY через переменную окружения или config.py.
Шаблон config.py
  # Алиасы и «стоп-слова» ассистента
  VA_ALIAS = ("джарвис", "jarvis")
  VA_TBR = ("скажи", "покажи", "запусти", "сделай")
  
  # Пути к моделям
  VOSK_MODEL_PATH = "assets/models/vosk/ru"
  PORCUPINE_ACCESS_KEY = "pv_xxx..."         # ваш ключ Porcupine
  PORCUPINE_KEYWORD_PATH = "assets/models/porcupine/jarvis_ru.ppn"
  
  # Аудио
  SOUNDS_DIR = "assets/sounds"
  
  # Ключи
  OPENAI_API_KEY = "sk-..."                  # или читайте из os.environ
  
  # Дроны / сеть
  DRONE_IPS = ["192.168.0.120", "192.168.0.121"]
  HOME_POINT = (0.0, -50.0, 0.0)             # точка RTB по умолчанию
  📡 Настройка сети Tello
  Для перевода Tello в режим AP с вашим SSID/паролем:
  python skynet.py --ssid <YOUR_SSID> --pwd <YOUR_PASS> --ip 192.168.10.1 --port 8889
  Задайте IP-адреса дронов в drone_manager.py или config.py и убедитесь, что ПК находится в одной подсети.

▶️ Запуск
    1) Голосовой ассистент
    python main.py
    Скажите ключевую фразу (например, «джарвис»), затем команду — из commands.yaml (взлёт/посадка/поток/миссии).
    2) Оркестратор миссии (без голоса)
    python mission/example_run.py
    Сценарий продемонстрирует построение планов для 2 БПЛА и выполнение цикла с безопасностью/RTB (вместо реального полёта требуется реализация телеметрии в mission_runner.py — методы _telemetry, _flight_cmd_set_velocity).

🧠 Как это работает
  1.   Голос → Команда. Активация Porcupine → распознавание Vosk → va_responder.py ищет совпадение в commands.yaml.
  2.   Команда → План миссии. Планировщик формирует задачи/зону (через LLM или пресеты).
  3.   План → Распределение. cbba_lite.py раздаёт подзадачи между дронами.
  4.   Маршруты покрытия. coverage_hybrid.py строит полосы с адаптивным шагом.
  5.   Полёт и безопасность. avoidance_reactive.py защищает локальное движение (TTC/клиренс, recovery).
  6.   Энергия/связь. energy_rtb.py контролирует запас и качество канала, упрощает план или включает RTB/LZ.
  7.   Ребаланс. Если один дрон уходит в RTB, незавершённые сегменты добирают соседи (CBBA-lite повторно).

🧪 Тесты/эмуляция
  Для стендовых проверок реализуйте заглушки телеметрии в mission_runner.py и подавайте синтетические препятствия/ветер/разряд.
  Для реальных полётов используйте безопасные зоны, включите ограничители (высота/коридоры), проверьте emergency stop.

Краткое описание каждого файла
  main.py — главный цикл: активация ключевым словом, распознавание речи (Vosk), реакция на команды через VAResponder, запуск сценариев миссии, голосовая обратная связь.
  audio_manager.py — работа с микрофоном/потоками аудио (PvRecorder), Porcupine wake word, Vosk ASR, воспроизведение коротких звуков, управление системной громкостью (PyCaw).
  tts.py — синтез речи (Silero: модель ru_v3), проигрывание речи в реальном времени.
  gpt_integration.py — LLM-помощник: формирование ответов/подсказок и сценариев миссий по голосовым командам.
  va_responder.py — нормализация распознанного текста, сопоставление с commands.yaml, вызов действий: звуки, системные команды, сценарии полёта, запуск миссий.
  commands.yaml — конфигурация голосовых команд: алиасы, триггеры, шаблоны параметров.
  drone_manager.py — пул подключений к DJI Tello, список IP, команды (взлёт/посадка/движение/поток), трекинг объектов, базовая безопасность.
  obstacle_avoidance.py — простая схема визуального избегания (границы/контуры), «разворот от стенки», RC-команды.
  tello_command.py — демонстрация TelloSwarm (подключение двух дронов, краткие фигуры, посадка).
  skynet.py — утилита настройки AP-режима Tello: SSID/пароль/адрес, работает через UDP.
  requirements.txt — фиксированный список Python-зависимостей.
  Report_201_13.06.2025.pdf — сопроводительный документ (внутренний).
 Добавляемые (без правок существующих):
  mission/mission_orchestrator.py — этап планирования: распределение задач (CBBA-lite) + построение покрытия (Hybrid Coverage) → пути для каждого дрона.
  mission/mission_runner.py — цикл выполнения: локальная безопасность (Reactive VO), мониторинг энергии/связи и решения RTB/LZ, перераспределение незавершённых сегментов.
  mission/example_run.py — пример запуска оркестратора на 2 БПЛА с набором задач.
  algorithms/schemas.py — типы данных (Point, Waypoint, Task, DroneState).
  algorithms/cbba_lite.py — аукцион CBBA-lite (≤5 раундов, tie-break по ID).
  algorithms/coverage_hybrid.py — генератор полос покрытия с адаптивным шагом по приоритету.
  algorithms/avoidance_types.py — структуры для обхода препятствий (Obstacle, вход/выход агента).
  algorithms/avoidance_reactive.py — Reactive VO + Dead-Wall (TTC/клиренс, look-and-turn recovery).
  algorithms/avoidance_vision_adapter.py — перевод детекций из видео в препятствия (упрощённо).
  algorithms/energy_rtb.py — оценка энергобаланса/связи и выбор режима (continue/simplify/RTB/LZ).
 Ожидаемые вспомогательные:
  config.py — токены/пути/алиасы/параметры (см. шаблон ниже).
  drone_utils.py — keep_alive(tello) и другие удобные утилиты.
  bottle_tracker.py — детектор/трекер объектов (используется в drone_manager.py).
  build_Fly.py — «мост» между голосовыми командами и миссиями (вызов оркестратора, запуск TelloSwarm).
  assets/models/ — модели Vosk/Porcupine; assets/sounds/ — короткие звуки ассистента.
  custom-commands/ — внешние утилиты для переключения аудиовыхода в Windows (если используете соответствующие команды).


# Как читать проект

Проект устроен слоями. Не нужно начинать с `dispatch.py`, gRPC или
Operator App: эти модули связывают много уже готовых механизмов и поэтому
понятны только после нижних слоёв.

## 1. Математика

Начните с каталога `fleet_manager/core/math`:

- `Vector2` — точка или вектор на плоскости;
- `Pose2D` — позиция и угол робота;
- `TimeInterval` — полуоткрытый интервал времени `[start, end)`;
- `Polygon2D` — выпуклый полигон и операции пересечения.

Эти классы не знают о роботах, заказах, ROS и интерфейсе. Они задают единые
правила вычислений: радианы, границы интервалов, допуски и геометрию.

## 2. Обычный поиск пути

Затем прочитайте `fleet_manager/core/search`. Там находится
общий детерминированный A*. Алгоритм работает с абстрактной задачей поиска:

1. задача выдаёт начальное состояние;
2. перечисляет соседей и стоимость перехода;
3. сообщает эвристику;
4. A* возвращает найденный путь.

MAPF-модули используют этот механизм и добавляют только правила конкретной
предметной области.

## 3. Планирование во времени

Рекомендуемый порядок:

1. `core/mapf/graph/traffic_graph_models.py` — граф движения;
2. `core/mapf/common/reservations.py` — занятость вершин и рёбер во времени;
3. `core/mapf/sipp/sipp.py` — маршрут одного робота между безопасными интервалами;
4. `core/mapf/rolling/rolling_sipp.py` — планирование нескольких роботов по частям;
5. `core/mapf/cbs/cbs_high_level.py` — локальное разрешение сложного конфликта;
6. `core/mapf/fleet/fleet_planner.py` — выбор алгоритма и построение траекторий.

Файлы-фасады намеренно короткие. Модели, подготовка запроса, сам алгоритм и
формирование результата находятся в соседних файлах с говорящими именами.
Это позволяет читать один уровень абстракции за раз.

Внутри CBS порядок чтения такой:

```text
cbs_models       данные и ограничения
      ↓
cbs_setup        проверка и нормализация запроса
      ↓
cbs_low_level    путь одного робота с ограничениями
      ↓
cbs_conflicts    поиск первого конфликта
      ↓
cbs_tree         узлы и очередь дерева ограничений
      ↓
cbs_high_level   координация поиска
```

## 4. Управление движением и трафиком

`manager/movement/motion.py` объединяет части управления live-движением:

- шаг движения и завершение сегмента;
- кинематика;
- проверка столкновений и атомарный откат;
- retreat;
- runtime replanning.

Трафик работает как последовательный конвейер:

```text
пространственный маршрут
        ↓
traffic zone / controlled corridor admission
        ↓
rolling SIPP reservations
        ↓
локальный CBS для связанного конфликта
        ↓
runtime recovery при длительной остановке
```

Каждый следующий механизм включается только тогда, когда более дешёвый
механизм не решил задачу.

Координатор deadlock тоже является фасадом. Отдельные компоненты отвечают за
граф ожидания, приоритет, leases, владение коридором, выбор эвакуационной
точки, установку escape-маршрута и разрыв цикла. Поэтому при изменении одного
правила не нужно читать весь recovery-механизм.

Три recovery-конвейера удобно читать по именам стадий:

```text
deadlock geometry
  edge variants → orientation selection → swept motion samples → blocker

deadlock activation
  candidate → blocked edges → graph escape / replan / reverse retreat → latch

rolling collapse
  stopped cohort → dependency graph → free sink → vacancy Dijkstra → prefetch
```

Установка graph escape является транзакцией: сначала обычный planner строит
траекторию, затем проверяется текущая геометрия тел, освобождаются устаревшие
corridor leases и только после этого одновременно меняются заказ и робот.

В `manager/tasks` stationary recovery устроен аналогично: causal episode хранит
signature и visited pockets, отдельный ограниченный поиск доказывает graph cut,
а отдельный commit создаёт внутренний `traffic_clearance` order. Ручное
возвращение робота после free-drive разделено на генерацию sampled motion,
collision audit и commit; публичный `manager.plan()` только координирует эти
стадии.

## 5. Runtime и Operator App

После алгоритмов переходите к прикладному слою:

1. `fleet_manager/manager/manager.py` собирает Fleet Manager,
   а `fleet_manager/manager/state.py` хранит контейнеры состояния;
2. `fleet_manager/runtime` подключает simulation или gRPC;
3. `operator_app/core/fleet_manager.py` формирует данные для Operator UI;
4. `operator_app/core/state.py` владеет менеджерами и рабочими каталогами;
5. `operator_app/web/handler.py` маршрутизирует HTTP;
6. `operator_app/web/socket_handlers.py` ведёт WebSocket-сеансы;
7. `operator_app/web/websocket.py` кодирует и разбирает кадры протокола.

HTTP и WebSocket не двигают симуляцию. Реальный и симуляционный менеджеры
имеют собственные управляемые runtime-циклы.

`manager.py`, `motion.py`, `traffic/routing.py`, `traffic/coordinator.py` и
`tasks/dispatch.py` — родительские композиционные классы. Они сохраняют
стабильный публичный API, но почти не содержат алгоритмов. Конкретная логика
лежит в небольших mixin-компонентах рядом:

```text
FleetManagerCore
├── состояние и настройки
├── команды, роботы, маршруты и snapshots
├── dispatch заказов и результаты планировщика
├── traffic admission / routing / recovery
└── motion step / safety / retreat / replanning
```

`operator_app/core/fleet_manager.py` — web-facing слой оркестрации. Работа с картами,
контекстом менеджера, ручным управлением, snapshot-ответами и benchmark
выполняется отдельными `fleet_*` компонентами в той же директории. Offline
differential/performance guard находится вне production core:
`operator_app/benchmarking/operator_fleet_refactor_guard.py`.

После отдельного refactor-коммита guard запускается так:

```bash
python3 -m operator_app.benchmarking.operator_fleet_refactor_guard \
  --legacy-ref HEAD^
```

`operator_app/core/state.py` устроен так же: реестр, управление роботами,
карты роботов, Fleet Manager API, карты флота и владение runtime находятся в
отдельных `state_*` модулях.

Канонический gRPC/ROS runtime разделён на lifecycle, control, maps, SLAM,
parameters и ROS helpers. Роботный пакет
`sim_robot/ws/src/robot_grpc_api` имеет такие же локальные компоненты, но не
зависит от исходников Fleet Manager и может разворачиваться отдельно.

Локальный контур движения робота находится в
`sim_robot/ws/src/robot_planner/robot_planner/executor.py`. Его лучше читать
сверху вниз через математические модели `RouteControlParameters`,
`RouteProgress` и `RouteSteeringState`: projection пути, reservation gate,
проверка arrival, steering errors и итоговая velocity command. Параметры
кэшируются до атомарной замены словаря при hot reload; геометрия вычисляется
заново на каждом control tick.

## Где вносить изменения

- Новая формула или геометрическая операция — `core/math`.
- Новый общий алгоритм поиска — `core/search`.
- Изменение SIPP/CBS/MAPF — соответствующий компонент в `core/mapf`.
- Чистая геометрия коридоров, collision math и wait graph — `core/traffic`.
- Live-политика очередей, коридоров и восстановления — `manager/traffic`.
- Команда флота или жизненный цикл робота — `manager` или
  `manager/tasks`.
- ROS/gRPC-преобразование — `fleet_manager/runtime`.
- HTTP-маршрут или WebSocket — `operator_app/web`.
- Визуальное поведение браузера — `operator_app/web/static/js`.

Не добавляйте UI, файловый кэш или конкретный gRPC-клиент внутрь
математического и планирующего слоя.

## Как безопасно менять алгоритм

Для алгоритмической правки нужны четыре проверки:

1. unit-тест нового компонента;
2. детерминированное сравнение со старой реализацией на фиксированном seed;
3. проверка инвариантов безопасности;
4. benchmark горячего пути.

Совпадение каждой точки маршрута полезно во время рефакторинга. Для новой
функциональности главные инварианты — отсутствие столкновений, корректные
резервации, достижение цели и ограниченное время планирования.

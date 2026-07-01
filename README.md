# warehouse_robot

ROS 2 + web stack for a warehouse robot simulator/operator workflow.

Main runtime split:

- `sim_robot/ws/src/robot_planner` - route planning and execution ROS 2 node.
- `sim_robot/ws/src/robot_status` - robot status ROS 2 node.
- `sim_robot/ws/src/robot_map_manager` - robot map state/load/sync ROS 2 services.
- `sim_robot/ws/src/robot_grpc_api` - local ROS 2 robot gRPC API backed by robot topics/services.
- `operator_app/robot_grpc_api` - local Operator App copy of the robot gRPC client contract.
- `fleet_manager/robot_grpc_api` - local Fleet Manager copy of the robot gRPC client contract.
- `operator_app` - desktop/web operator application. Browser talks to it over HTTP/WebSocket; robot communication uses gRPC.
- `fleet_manager/web_simulator` - web/no-ROS fleet simulator runtime.
- `fleet_manager` - Fleet Manager map/MAPF/runtime code.

Map/planning ownership:

- `fleet_manager/route_core` owns Fleet Manager map loading/edit exchange for `fleet_manager/map_data`.
- `fleet_manager/mapf` owns Fleet Manager MAPF and space-time planning.
- `sim_robot/ws/src/robot_planner/robot_planner/route_core` owns robot-side map/route loading.
- Operator App stores its local editable map cache independently and only synchronizes by push/pull.
- `robot_grpc_api` is only the network API contract/runtime; it does not own maps or MAPF.

Runtime transport rule:

- Browser/site <-> Operator App: HTTP/WebSocket.
- Operator App/Fleet Manager <-> robots: gRPC over TCP.
- Inside each robot: local ROS 2/Nav2/topics/services, not exposed to the server.

Install gRPC runtime on the operator/server and on each robot:

```bash
sudo apt install python3-grpcio
```

## Build Robot Packages

```bash
cd ~/warehouse_robot/sim_robot/ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_msgs robot_planner robot_status robot_map_manager robot_grpc_api robot_launch
source install/local_setup.bash
```

## Run Simulator

```bash
cd ~/warehouse_robot/sim_robot/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch stage_ros2 stage.launch.py enforce_prefixes:=false one_tf_tree:=false
```

If needed, start Nav2/AMCL in a separate terminal:

```bash
cd ~/warehouse_robot/sim_robot/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch nav2 nav2_launch.py
```

## Run Robot Stack

This starts status, route execution, map manager, and native gRPC robot API nodes.

```bash
cd ~/warehouse_robot/sim_robot/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_LOG_DIR=/tmp/ros_logs
ros2 launch robot_launch launch.py
```

Useful launch overrides:

```bash
ros2 launch robot_launch launch.py map_dir:=/path/to/map.smap params:=/path/to/params.yaml
```

## Run Operator App

Run this on the operator computer. Add robots by IP and gRPC port, default `50051`.

```bash
cd ~/warehouse_robot
python3 serve_operator.py --open
```
Да, ты повторяешь ядро Roboshop Pro, но пока не весь Roboshop. Сейчас у тебя есть рабочий контур: подключение робота, статус, teleop, stop/cancel, маршрут до LM, scan overlay, push/pull/load карт, параметры, простой редактор LM/edges и Fleet queue/orders. Это видно по gRPC API: [robot_api.proto (line 5)](/home/kaisar/warehouse_robot/sim_robot/ws/src/robot_grpc_api/robot_grpc_api/proto/robot_api.proto:5), панели управления: [index.html (line 100)](/home/kaisar/warehouse_robot/operator_app/static/index.html:100), fleet queue: [index.html (line 141)](/home/kaisar/warehouse_robot/operator_app/static/index.html:141), редактору карты: [map-editor.html (line 81)](/home/kaisar/warehouse_robot/operator_app/static/map-editor.html:81).
Чего не хватает до Roboshop Pro
SLAM и rawmap workflow: 2D/3D build map, scan rawmap, download/load rawmap, offline mapping, сохранение результата как новой карты. У тебя сейчас есть импорт/редактирование существующей .smap/ROS map, но не создание карты роботом с нуля.
Серьезный map editor: шум/ластик, thinning walls, rotate/origin transform, stitch/correct/check map, batch edit, split/merge/smooth paths, road width/curvature, site alignment, advanced areas, QR/PGV, reflector areas, forbidden/block/bypass zones, DI/DO areas. Сейчас редактор в основном Select/LM/Edge.
Relocation и control ownership: Roboshop имеет preempt/release control, relocate, confirm localization, pause/continue navigation. У тебя есть stop/cancel/teleop/execute, но нет полноценного жизненного цикла управления роботом.
Task chain/action system: в Roboshop есть цепочки задач: path nav, translation, rotation, custom action, fork/jack/hook, play sound, wait DI, set DO, groups, timeout, loop, suspend, execute next. У тебя fleet order сейчас ближе к “очередь целей LM”, без action graph.
I/O и периферия: DI/DO monitor/control, PLC/modbus, charger, fork/jack/hook/roller, PGV, камеры/vision server, audio management, light/buzzer behavior. В статусе батарея есть, но действий зарядки/докинга и железных actuator-команд нет.
Advanced configuration: Wi-Fi/DHCP/static IP, robot files/model files, калибровки gyro/encoder/laser-odom, factory/debug функции. Это большой кусок Roboshop, которого у тебя почти нет.
Диагностика и сервисность: alarm codes, detailed hardware status, running curves, logs/history, parameter compare/export/default restore/temp vs permanent changes. У тебя есть params JSON/table, но не полноценная сервисная панель.
Совместимость с настоящим Roboshop/SRC API: Roboshop в мануале опирается на набор robot TCP API портов вроде 19204..19310; у тебя свой gRPC слой. Для своего ROS/Nav2 стека это нормально. Для замены Roboshop на реальном AIVISION роботе нужен adapter к их протоколам.
Что я бы добавлял первым
Relocate / confirm localization / pause-resume / control lock.
Map validation + расширение редактора: advanced areas, path properties, site types.
Task chain model: steps, groups, timeouts, actions, DI/DO.
Charging + IO abstraction, хотя бы интерфейсы без реального железа.
Диагностика: alarms, event history, logs, health panels.
И коротко: ты уже повторяешь Operator + Fleet + Map Sync часть. До Roboshop Pro больше всего не хватает SLAM, сервисной диагностики, task-chain/action layer и железных интеграций.

SLAM и rawmap workflow: 2D/3D build map, scan rawmap, download/load rawmap, offline mapping, сохранение результата как новой карты. У тебя сейчас есть импорт/редактирование существующей .smap/ROS map, но не создание карты роботом с нуля.

в нашей версии будет только 2d slam. 
теперь это
я скачал это
sim_robot/ws/src/slam_toolbox
давай будем исползовать его для slam 2d
смотри я запускаю 2d slam 
далее уменя выходят парамтеры  slam ,со стандартными значениями я их редактирую либо просто жму начать

далее должно начаться  sim_robot/ws/src/slam_toolbox/launch/online_async_launch.py
робот едет и делает карту
И, отопбражает на сайте
нужно как то топик /map передавать через gRPC в operator app
и дальше оно дложно отображаться на сайте, я должен видть робота,
строющуюся карту, путь каторый проехал линией как след.

Оно работет.
потом я могу нажать на Done. написать название карты
и наш робот его  map manager
олжен создать папку  в папке sim_robot/ws/src/robot_map_manager/maps_out с названием карты картой и туда закинуть pgm + .yaml его
создать так же пустые все файлы  .operator_meta.json graph_edges_lengths.yaml 
и тд. Ну то есть по сути новая smap карта.

и дальше operator app олжен сделать pull ее к себе в operator_app/operator_data в конкртного робота
далее я уже смогу эту карту редактировать и тд.

перед работай объясни как понял, задача очень сложная поидее

Show more

Tuesday 11:22 PM






Worked for 57s



Понял так.
Мы добавляем не “редактор карты”, а новый workflow: создание новой 2D-карты роботом с нуля через slam_toolbox.
Текущая система уже умеет работать с готовой .smap-картой: pull/push/load bundle, редактирование графа, landmarks, map manager. Но SLAM-сессии как жизненного цикла пока нет.
Нужный Workflow
В Operator App появляется режим 2D SLAM.
Пользователь выбирает робота и жмет что-то типа Start 2D SLAM.
Открывается панель параметров SLAM со значениями из:
[mapper_params_online_async.yaml](/home/kaisar/warehouse_robot/sim_robot/ws/src/slam_toolbox/config/mapper_params_online_async.yaml)
Пользователь может отредактировать параметры или оставить дефолтные.
Жмет Start.
На роботе через gRPC запускается:
[online_async_launch.py](/home/kaisar/warehouse_robot/sim_robot/ws/src/slam_toolbox/launch/online_async_launch.py)
slam_toolbox начинает публиковать /map.
Robot gRPC API должен стримить эту live /map в Operator App.
На сайте показываем:строящуюся occupancy grid карту,
текущую позу робота,
след/траекторию, где робот уже проехал,
возможно scan overlay, как сейчас.

Когда карта готова, пользователь жмет Done.
Вводит имя новой карты.
Robot map manager создает новую папку:
   sim_robot/ws/src/robot_map_manager/maps_out/<map_name>.smap/
Туда сохраняются:<map_name>.pgm
<map_name>.yaml
пустые/стартовые LMs.yaml
пустые/стартовые graphs.yaml
пустой/стартовый graph_edges_lengths.yaml
primitives_lengths.csv
.operator_meta.json
smap_summary.json

После сохранения новая карта становится обычной .smap-картой твоей системы.
Operator App делает pull этой карты в operator_app/operator_data/<robot>/maps/....
Дальше ты уже редактируешь ее как обычную карту: landmarks, graph edges, lengths и т.д.
Как это встроить технически
Я бы добавлял новый слой рядом с существующим map lifecycle:
В robot_api.proto добавить SLAM RPC:
GetSlamDefaults
StartSlam
WatchSlamMap
StopSlam или CancelSlam
FinishSlam

В robot gRPC runtime добавить ROS2 bridge:
запуск slam_toolbox launch-файла,
подписка на /map как nav_msgs/OccupancyGrid,
сохранение карты через slam_toolbox / map saver,
создание .smap папки через map manager.

В Operator App добавить UI:
панель 2D SLAM,
редактор параметров,
live map renderer,
trail renderer,
кнопки Start, Cancel, Done.

Важный момент
/map лучше не пытаться сразу превращать в постоянный .smap. Во время SLAM это live-карта. Ее надо стримить отдельно как live occupancy grid. А уже на Done сохранить в .pgm + .yaml, создать структуру .smap, затем сделать pull как обычную карту.
Как бы я делал по этапам
Backend contract: добавить gRPC сообщения/RPC для SLAM lifecycle.
Robot runtime: подписка на /map, конвертация OccupancyGrid в компактный payload.
Operator App: live preview карты + pose + trail.
Start SLAM: запуск online_async_launch.py с временным params yaml.
Finish SLAM: сохранить .pgm/.yaml, создать .smap директорию и пустые graph/meta файлы.
Auto pull новой карты в operator workspace.
Самый правильный первый шаг: сделать минимальный SLAM session contract + live /map stream, без сохранения. Тогда мы сразу увидим строящуюся карту на сайте и поймем, что канал работает. Потом уже добавим Done и генерацию .smap.

Tuesday 11:23 PM






ок, во время slam должен быть доступен teleop через wasd как сейчас
потом когда я нажимаю one должен появится окно с закгрузкой с 0 до 100 процентов, чтобы было понятно что идет процесс какой то.



Tuesday 11:26 PM






Worked for 27m 54s



Готово. Добавил первый рабочий слой для 2D SLAM workflow:
Start 2D / Done / Cancel в отдельной группе SLAM.
Start открывает параметры slam_toolbox с дефолтами, можно редактировать JSON и запускать.
SLAM запускает online_async_launch.py, слушает live /map через gRPC/WebSocket и рисует строящуюся карту на сайте.
Рисуется след робота поверх карты.
WASD teleop остается доступен параллельно, я его не завязывал на SLAM-состояние.
Done спрашивает имя карты и показывает окно прогресса 0-100%, сохраняет .pgm/.yaml, создает пустые .smap sidecar-файлы, грузит карту на роботе и подтягивает ее в operator data.
Добавлены gRPC/HTTP/WebSocket endpoints для SLAM во всех зеркалах operator_app, sim_robot, fleet_manager.
Проверил:
python3 -m compileall operator_app fleet_manager sim_robot/ws/src/robot_grpc_api sim_robot/ws/src/launch проходит.
Proto descriptor видит все новые SLAM RPC.
Operator server поднят и отвечает: http://127.0.0.1:8781/
/health и /api/robots?probe=0 работают.
Не смог сделать node --check, потому что node не установлен в окружении. Реальный SLAM launch через gRPC тоже не дергал: сохраненный робот сейчас online: false, нужен запущенный robot API/ROS.

Web previewWebsiteOpen in browserOpen in

Edited 12 files+1,726-4Review changesUndoReview

sim_robot/ws/src/robot_grpc_api/robot_grpc_api/proto/robot_api.protosim_robot/ws/src/robot_grpc_api/robot_grpc_api/proto/robot_api.proto+98-0
operator_app/robot_grpc_api/proto/robot_api_pb2_grpc.pyoperator_app/robot_grpc_api/proto/robot_api_pb2_grpc.py+90-0
sim_robot/ws/src/robot_grpc_api/robot_grpc_api/server.pysim_robot/ws/src/robot_grpc_api/robot_grpc_api/server.py+142-0
Show 9 more files
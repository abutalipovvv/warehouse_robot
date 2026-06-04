#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path
from threading import Thread

PROJECT_ROOT = Path(__file__).resolve().parent
ROBOT_WORKSPACE = PROJECT_ROOT / "robot"


def _bootstrap_python_paths() -> None:
    major = sys.version_info.major
    minor = sys.version_info.minor
    candidates = [
        Path(f"/opt/ros/jazzy/lib/python{major}.{minor}/site-packages"),
    ]
    install_root = ROBOT_WORKSPACE / "install"
    if install_root.exists():
        candidates.extend(sorted(install_root.glob(f"*/lib/python{major}.{minor}/site-packages")))
    candidates.extend(
        [
            ROBOT_WORKSPACE / "ws" / "src",
            ROBOT_WORKSPACE / "ws" / "src" / "robot_http_server",
            ROBOT_WORKSPACE / "ws" / "src" / "ros2_http_client",
            ROBOT_WORKSPACE / "ws" / "src" / "robot_planner",
            ROBOT_WORKSPACE / "ws" / "src" / "robot_planner" / "robot_planner",
            ROBOT_WORKSPACE / "ws" / "src" / "robot_status",
            ROBOT_WORKSPACE / "ws" / "src" / "robot_map_manager",
        ]
    )
    for candidate in candidates:
        candidate_str = str(candidate.resolve())
        if candidate.exists() and candidate_str not in sys.path:
            sys.path.insert(0, candidate_str)


_bootstrap_python_paths()

from robot_http_server.server import RobotHttpApiBridge, parse_args, resolve_map_dir, serve_http_server


def main() -> None:
    args = parse_args()
    map_dir = resolve_map_dir(args.map_dir)
    params_path = args.params.resolve()

    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from ros2_http_client import RobotRosClient

    rclpy.init(args=None)
    ros_client = RobotRosClient(
        robot_id=args.robot_id,
        map_dir=map_dir,
        params_path=params_path,
        status_topic=args.status_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        plan_service_name=args.plan_service,
        execute_service_name=args.execute_service,
        cancel_service_name=args.cancel_service,
        map_state_service_name=args.map_state_service,
        map_load_service_name=args.map_load_service,
    )
    http_bridge = RobotHttpApiBridge(
        robot_id=args.robot_id,
        ros_client=ros_client,
    )

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(ros_client.node)
    spinner = Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        ros_client.sync_active_map_context()
    except Exception:
        pass

    try:
        serve_http_server(
            bridge=http_bridge,
            robot_id=args.robot_id,
            host=args.host,
            port=args.port,
            open_browser=args.open,
        )
    finally:
        executor.shutdown()
        ros_client.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

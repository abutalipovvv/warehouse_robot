from __future__ import annotations

from pathlib import Path
from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .client import RobotRosClient
from .server import RobotHttpApiBridge, parse_args, resolve_map_dir, serve_http_server


def main() -> None:
    args = parse_args()
    map_dir = resolve_map_dir(Path(args.map_dir))
    params_path = Path(args.params).resolve()

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

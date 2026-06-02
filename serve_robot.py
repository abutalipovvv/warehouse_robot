#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
from pathlib import Path
from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_http_server.server import RobotHttpApiBridge, parse_args, resolve_map_dir, serve_http_server
from ros2_http_client import RobotRosClient


def main() -> None:
    args = parse_args()
    map_dir = resolve_map_dir(args.map_dir)
    params_path = args.params.resolve()

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
    )
    http_bridge = RobotHttpApiBridge(
        robot_id=args.robot_id,
        map_id=ros_client.map_id,
        ros_client=ros_client,
    )

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(ros_client.node)
    spinner = Thread(target=executor.spin, daemon=True)
    spinner.start()

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

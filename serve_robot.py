#!/usr/bin/env python3
import sys
from pathlib import Path
from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor


PROJECT_ROOT = Path(__file__).resolve().parent
WORKSPACE_PACKAGE_ROOTS = [
    PROJECT_ROOT / "robot" / "ws" / "src" / "robot_planner",
    PROJECT_ROOT / "robot" / "ws" / "src" / "robot_status",
]
for package_root in reversed(WORKSPACE_PACKAGE_ROOTS):
    package_root_str = str(package_root)
    if package_root.exists() and package_root_str not in sys.path:
        sys.path.insert(0, package_root_str)

from robot_http_api.server import RobotHttpApiBridge, parse_args, resolve_map_dir, serve_http_server
from robot_http_api.ros_client import RobotRosClient
from robot_planner import RobotRuntime, RobotTrajectoryPlanner, RouteExecutorNode, RoutePlannerNode
from robot_status import RobotStatusNode


def main() -> None:
    args = parse_args()
    map_dir = resolve_map_dir(args.map_dir)
    params_path = args.params.resolve()
    route_planner = RobotTrajectoryPlanner(map_dir=map_dir, params_path=params_path)
    runtime = RobotRuntime(robot_id=args.robot_id, map_id=route_planner.map_id)

    rclpy.init(args=None)
    status_node = RobotStatusNode(
        runtime=runtime,
        route_planner=route_planner,
        amcl_topic=args.amcl_topic,
        odom_topic=args.odom_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        status_topic=args.status_topic,
    )
    planner_node = RoutePlannerNode(
        route_planner=route_planner,
        service_name=args.plan_service,
    )
    executor_node = RouteExecutorNode(
        runtime=runtime,
        route_planner=route_planner,
        cmd_vel_topic=args.cmd_vel_topic,
        status_topic=args.status_topic,
    )
    ros_client = RobotRosClient(
        runtime=runtime,
        status_topic=args.status_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        plan_service_name=args.plan_service,
        execute_service_name=args.execute_service,
        cancel_service_name=args.cancel_service,
    )
    http_bridge = RobotHttpApiBridge(
        runtime=runtime,
        route_planner=route_planner,
        params_path=params_path,
        ros_client=ros_client,
    )

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(status_node)
    executor.add_node(planner_node)
    executor.add_node(executor_node)
    executor.add_node(ros_client.node)
    spinner = Thread(target=executor.spin, daemon=True)
    spinner.start()

    try:
        serve_http_server(
            bridge=http_bridge,
            route_planner=route_planner,
            robot_id=args.robot_id,
            host=args.host,
            port=args.port,
            open_browser=args.open,
        )
    finally:
        executor.shutdown()
        status_node.destroy_node()
        planner_node.destroy_node()
        executor_node.destroy_node()
        ros_client.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

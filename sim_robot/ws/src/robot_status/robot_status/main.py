from __future__ import annotations

import argparse
from pathlib import Path

import rclpy

from robot_planner import RobotTrajectoryPlanner

from .node import RobotStatusNode


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run robot status aggregation node.")
    parser.add_argument("--map-dir", required=True, type=Path)
    parser.add_argument("--params", required=True, type=Path)
    parser.add_argument("--robot-id", default="robot1")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--amcl-topic", default="/amcl_pose")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--battery-topic", default="/bms")
    parser.add_argument("--status-topic", default="/robot_status")
    parser.add_argument("--executor-status-topic", default="/route/executor_state")
    parser.add_argument("--load-map-service", default="/status/load_map")
    args, _unknown = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    route_planner = RobotTrajectoryPlanner(
        map_dir=args.map_dir,
        params_path=args.params,
    )
    rclpy.init(args=None)
    node = RobotStatusNode(
        robot_id=args.robot_id,
        route_planner=route_planner,
        amcl_topic=args.amcl_topic,
        odom_topic=args.odom_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        battery_topic=args.battery_topic,
        status_topic=args.status_topic,
        executor_status_topic=args.executor_status_topic,
        load_map_service_name=args.load_map_service,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

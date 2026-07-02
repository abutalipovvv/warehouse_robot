from __future__ import annotations

import argparse
import signal
import time

from .contracts import DEFAULT_GRPC_PORT
from .ros_runtime import RosRobotRuntime
from .server import serve_robot_api


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Native robot gRPC API backed by local ROS 2 topics/services.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_GRPC_PORT)
    parser.add_argument("--robot-id", default="robot1")
    parser.add_argument("--robot-name", default="")
    parser.add_argument("--namespace", default="")
    parser.add_argument("--status-topic", default="/robot_status")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--initial-pose-topic", default="/initialpose")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--go-to-lm-topic", default="/go_to_lm")
    parser.add_argument("--plan-service", default="/route/plan")
    parser.add_argument("--execute-service", default="/route/execute")
    parser.add_argument("--cancel-service", default="/route/cancel")
    parser.add_argument("--route-pause-service", default="/route/pause")
    parser.add_argument("--route-load-map-service", default="/route/load_map")
    parser.add_argument("--status-load-map-service", default="/status/load_map")
    parser.add_argument("--map-state-service", default="/robot/maps/state")
    parser.add_argument("--map-load-service", default="/robot/maps/load")
    parser.add_argument("--map-list-service", default="/robot/maps/list")
    parser.add_argument("--map-get-bundle-service", default="/robot/maps/get_bundle")
    parser.add_argument("--map-put-bundle-service", default="/robot/maps/put_bundle")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--slam-save-map-service", default="/slam_toolbox/save_map")
    parser.add_argument("--reset-odom-service", default="/reset_odom")
    parser.add_argument("--slam-params-file", default="")
    parser.add_argument("--slam-launch-file", default="")
    parser.add_argument("--params", default="")
    args, _unknown_ros_args = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    runtime = RosRobotRuntime(
        robot_id=args.robot_id,
        robot_name=args.robot_name or args.robot_id,
        namespace=args.namespace,
        status_topic=args.status_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        odom_topic=args.odom_topic,
        initial_pose_topic=args.initial_pose_topic,
        scan_topic=args.scan_topic,
        map_frame=args.map_frame,
        base_frame=args.base_frame,
        go_to_lm_topic=args.go_to_lm_topic,
        plan_service_name=args.plan_service,
        execute_service_name=args.execute_service,
        cancel_service_name=args.cancel_service,
        route_pause_service_name=args.route_pause_service,
        route_load_map_service_name=args.route_load_map_service,
        status_load_map_service_name=args.status_load_map_service,
        map_state_service_name=args.map_state_service,
        map_load_service_name=args.map_load_service,
        map_list_service_name=args.map_list_service,
        map_get_bundle_service_name=args.map_get_bundle_service,
        map_put_bundle_service_name=args.map_put_bundle_service,
        map_topic=args.map_topic,
        slam_save_map_service_name=args.slam_save_map_service,
        reset_odom_service_name=args.reset_odom_service,
        slam_params_file=args.slam_params_file or None,
        slam_launch_file=args.slam_launch_file or None,
        params_path=args.params,
    )
    server = serve_robot_api(runtime, host=args.host, port=args.port)
    stop = False

    def _handle_signal(signum, frame) -> None:
        del signum, frame
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)
    print(f"robot gRPC API {args.robot_id} listening on {args.host}:{args.port}", flush=True)
    try:
        while not stop:
            time.sleep(0.2)
    finally:
        server.stop(grace=1.0)
        runtime.close()


if __name__ == "__main__":
    main()

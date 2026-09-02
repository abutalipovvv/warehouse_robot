"""ROS node that is the sole publisher to the robot driver cmd_vel topic."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .config import load_gateway_settings
from .model import CommandDecision, MotionArbiter, MotionMode


class MotionGatewayNode(Node):
    def __init__(
        self,
        *,
        robot_id: str,
        params_path: Path | None,
        output_topic: str,
        route_topic: str,
        teleop_topic: str,
        nav2_topic: str,
        mode_topic: str,
        state_topic: str,
    ) -> None:
        super().__init__("robot_motion_gateway")
        self.robot_id = str(robot_id or "robot1")
        self.params_path = params_path
        self._params_mtime_ns: int | None = None
        self._settings = load_gateway_settings(params_path)
        self._arbiter = MotionArbiter(
            limits=self._settings.limits,
            timeouts=self._settings.timeouts,
        )
        self._output_pub = self.create_publisher(Twist, output_topic, 20)
        self._state_pub = self.create_publisher(String, state_topic, 10)
        self.create_subscription(
            Twist,
            route_topic,
            lambda message: self._on_command("route", message),
            20,
        )
        self.create_subscription(
            Twist,
            teleop_topic,
            lambda message: self._on_command("teleop", message),
            20,
        )
        self.create_subscription(
            Twist,
            nav2_topic,
            lambda message: self._on_command("nav2", message),
            20,
        )
        self.create_subscription(String, mode_topic, self._on_mode, 20)
        self._last_state_key: tuple[object, ...] | None = None
        self._last_watchdog_logged = False
        self._last_state_publish_at = 0.0
        self._timer = self.create_timer(
            1.0 / self._settings.publish_rate_hz,
            self._publish_output,
        )
        self.create_timer(1.0, self._reload_settings)
        self.get_logger().info(
            "ready: "
            f"robot={self.robot_id} output={output_topic} "
            f"inputs=[{route_topic}, {teleop_topic}, {nav2_topic}]"
        )

    def stop(self) -> None:
        self._arbiter.transition(MotionMode.IDLE, reason="shutdown")
        self._publish_twist(0.0, 0.0)
        self._publish_state(self._arbiter.decide(now=monotonic()), force=True)

    def _on_command(self, source: str, message: Twist) -> None:
        try:
            self._arbiter.accept(
                source,
                linear=float(message.linear.x),
                angular=float(message.angular.z),
                received_at=monotonic(),
            )
        except ValueError as exc:
            self.get_logger().warning(f"rejected {source} velocity command: {exc}")

    def _on_mode(self, message: String) -> None:
        try:
            mode, reason = _mode_payload(message.data)
            changed = self._arbiter.transition(mode, reason=reason)
        except ValueError as exc:
            self.get_logger().warning(f"rejected motion mode request: {exc}")
            return
        if changed:
            self._publish_twist(0.0, 0.0)
            self._last_watchdog_logged = False
            self.get_logger().info(
                f"motion mode: {self._arbiter.mode.value} ({self._arbiter.reason})"
            )

    def _publish_output(self) -> None:
        decision = self._arbiter.decide(now=monotonic())
        self._publish_twist(decision.linear, decision.angular)
        if decision.watchdog_stop and not self._last_watchdog_logged:
            self.get_logger().warning(
                f"motion watchdog stop: mode={decision.mode.value} "
                f"source={decision.active_source} reason={decision.reason}"
            )
        self._last_watchdog_logged = decision.watchdog_stop
        self._publish_state(decision)

    def _publish_twist(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self._output_pub.publish(message)

    def _publish_state(
        self,
        decision: CommandDecision,
        *,
        force: bool = False,
    ) -> None:
        now = monotonic()
        state_key = (
            decision.mode.value,
            decision.active_source,
            round(decision.linear, 4),
            round(decision.angular, 4),
            decision.watchdog_stop,
            decision.reason,
        )
        if (
            not force
            and state_key == self._last_state_key
            and now - self._last_state_publish_at < 0.5
        ):
            return
        payload = {
            "robotId": self.robot_id,
            "mode": decision.mode.value,
            "activeSource": decision.active_source,
            "linear": decision.linear,
            "angular": decision.angular,
            "watchdogStop": decision.watchdog_stop,
            "reason": decision.reason,
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self._state_pub.publish(message)
        self._last_state_key = state_key
        self._last_state_publish_at = now

    def _reload_settings(self) -> None:
        path = self.params_path
        if path is None:
            return
        try:
            mtime_ns = path.stat().st_mtime_ns
        except OSError:
            return
        if self._params_mtime_ns == mtime_ns:
            return
        settings = load_gateway_settings(path)
        self._arbiter.configure(
            limits=settings.limits,
            timeouts=settings.timeouts,
        )
        self._settings = settings
        self._params_mtime_ns = mtime_ns
        self.get_logger().info(
            "motion limits reloaded: "
            f"forward={settings.limits.max_forward_speed:.2f}m/s "
            f"backward={settings.limits.max_backward_speed:.2f}m/s "
            f"angular={settings.limits.max_angular_speed:.2f}rad/s"
        )


def _mode_payload(raw: str) -> tuple[MotionMode, str]:
    source = str(raw or "").strip()
    if not source:
        raise ValueError("motion mode is empty")
    if source.startswith("{"):
        try:
            payload = json.loads(source)
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid mode JSON: {exc}") from exc
        if not isinstance(payload, dict):
            raise ValueError("mode JSON must contain an object")
        mode = MotionMode.parse(payload.get("mode"))
        reason = str(payload.get("reason") or mode.value.lower())
        return mode, reason
    mode = MotionMode.parse(source)
    return mode, mode.value.lower()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Arbitrate route, teleop and Nav2 velocity commands.",
    )
    parser.add_argument("--robot-id", default="robot1")
    parser.add_argument("--params", type=Path, default=None)
    parser.add_argument("--output-topic", default="cmd_vel")
    parser.add_argument("--route-topic", default="motion/route_cmd_vel")
    parser.add_argument("--teleop-topic", default="motion/teleop_cmd_vel")
    parser.add_argument("--nav2-topic", default="motion/nav2_cmd_vel")
    parser.add_argument("--mode-topic", default="motion/mode")
    parser.add_argument("--state-topic", default="motion/state")
    args, _unknown = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    rclpy.init(args=None)
    node = MotionGatewayNode(
        robot_id=args.robot_id,
        params_path=args.params,
        output_topic=args.output_topic,
        route_topic=args.route_topic,
        teleop_topic=args.teleop_topic,
        nav2_topic=args.nav2_topic,
        mode_topic=args.mode_topic,
        state_topic=args.state_topic,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

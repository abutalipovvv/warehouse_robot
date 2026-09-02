"""Publish deterministic battery telemetry for Stage simulations."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import BatteryState


BMS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def fake_battery_state() -> BatteryState:
    """Build the stable battery payload shared by every simulated robot."""
    message = BatteryState()
    message.present = True
    message.voltage = 52.0
    message.current = -5.0
    message.percentage = 0.75
    message.temperature = 30.0
    message.power_supply_status = (
        BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
    )
    message.cell_voltage = [6.5] * 8
    message.cell_temperature = [30.0, 30.0]
    return message


class FakeBmsPublisher(Node):
    """Publish a stable BatteryState and exit cleanly with ROS shutdown."""

    def __init__(self) -> None:
        super().__init__('fake_bms_pub')
        self._publisher = self.create_publisher(
            BatteryState,
            '/bms',
            BMS_QOS,
        )
        self._message = fake_battery_state()
        self.create_timer(0.1, self._publish)

    def _publish(self) -> None:
        self._message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._message)


def main() -> None:
    rclpy.init(args=None)
    node = FakeBmsPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Battery source: publishes batteryStatus per message; contributes CHARGING / LOW_BATTERY."""
import logging
import math

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState

from iso21423_ros.mappings import battery_body

log = logging.getLogger(__name__)


class BatterySource:
    def __init__(self, node, publisher, params):
        self.node, self.pub = node, publisher
        self.low = float(params.get("low_threshold", 0.2))
        self._states = []  # swapped atomically; states() takes no lock
        node.create_subscription(BatteryState, params.get("topic", "/battery_state"), self._on_msg,
                                 qos_profile_sensor_data)

    def states(self):
        return list(self._states)

    def _on_msg(self, msg):
        s = []
        if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            s.append("CHARGING")
        if math.isfinite(msg.percentage) and msg.percentage < self.low:
            s.append("LOW_BATTERY")
        self._states = s
        try:
            self.pub.publish_battery_status(battery_body(msg))
        except Exception:
            log.exception("battery publish failed")
        self.node.status.refresh()

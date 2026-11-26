"""Global plan (deduped by positions) and local trajectory (throttled) sources."""
import logging
import time

from nav_msgs.msg import Path
from rclpy.qos import qos_profile_sensor_data

from iso21423_ros.mappings import plan_key, stamped_points

log = logging.getLogger(__name__)


class GlobalPlanSource:
    def __init__(self, node, publisher, params):
        self.node, self.pub = node, publisher
        self._last_key = None
        node.create_subscription(Path, params.get("topic", "/plan"), self._on_msg, qos_profile_sensor_data)

    def _on_msg(self, msg):
        try:
            key = plan_key(msg)
            if key == self._last_key:
                return
            self.pub.publish_global_plan(stamped_points(self.node.ccs, msg))
            self._last_key = key  # only after success, so a failed publish is retried
        except Exception:
            log.exception("global plan publish failed")


class LocalTrajectorySource:
    def __init__(self, node, publisher, params):
        self.node, self.pub = node, publisher
        rate = float(params.get("rate_hz", 2.0))
        if rate <= 0:
            raise ValueError(f"local trajectory rate_hz must be > 0, got {rate}")
        self._min_interval = 1.0 / rate
        self._last = None
        node.create_subscription(Path, params.get("topic", "/local_plan"), self._on_msg, qos_profile_sensor_data)

    def _on_msg(self, msg):
        now = time.monotonic()
        if self._last is not None and now - self._last < self._min_interval:
            return
        try:
            self.pub.publish_local_trajectory(stamped_points(self.node.ccs, msg))
            self._last = now
        except Exception:
            log.exception("local trajectory publish failed")

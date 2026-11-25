"""Odometry source: map->base TF at rate_hz plus twist from nav_msgs/Odometry."""
import logging

from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from iso21423_ros.mappings import odometry_body, yaw_from_quaternion

log = logging.getLogger(__name__)


class OdometrySource:
    def __init__(self, node, publisher, params):
        self.node, self.pub = node, publisher
        self.lost_after = float(params.get("lost_after_s", 2.0))
        self._lost = True  # plain attribute reads/writes: states() takes no lock
        self._linear = 0.0
        self._angular = 0.0
        node.create_subscription(Odometry, params.get("twist_topic", "/odom"), self._on_odom,
                                 qos_profile_sensor_data)
        node.create_timer(1.0 / float(params.get("rate_hz", 2.0)), self._tick)

    def states(self):
        if self._lost:
            return ["LOST"]
        if abs(self._linear) > 0.01:
            return ["FORWARD" if self._linear > 0 else "REVERSE"]
        return []

    def _on_odom(self, msg):
        self._linear = msg.twist.twist.linear.x
        self._angular = msg.twist.twist.angular.z
        self.node.status.refresh()

    def _tick(self):
        try:
            tf = self.node.tf_buffer.lookup_transform(self.node.map_frame, self.node.base_frame, Time())
            age = self.node.get_clock().now() - Time.from_msg(tf.header.stamp)
            self._lost = age > Duration(seconds=self.lost_after)
        except Exception:  # TransformException, or no data yet
            self._lost, tf = True, None
        if not self._lost:
            t, q = tf.transform.translation, tf.transform.rotation
            try:
                self.pub.publish_odometry(odometry_body(
                    self.node.ccs, t.x, t.y, yaw_from_quaternion(q), self._linear, self._angular))
            except Exception:
                log.exception("odometry publish failed")
        self.node.status.refresh()

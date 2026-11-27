# Copyright 2026 InOrbit, Inc.
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

"""Odometry source: map->base TF at rate_hz plus twist from nav_msgs/Odometry."""
import logging
import time

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
        rate = float(params.get("rate_hz", 2.0))
        if rate <= 0:
            raise ValueError(f"odometry rate_hz must be > 0, got {rate}")
        self._odom_at = None  # monotonic time of the last /odom message
        self._lost = True  # plain attribute reads/writes: states() takes no lock
        self._linear = 0.0
        self._angular = 0.0
        node.create_subscription(Odometry, params.get("twist_topic", "/odom"), self._on_odom,
                                 qos_profile_sensor_data)
        node.create_timer(1.0 / rate, self._tick)

    def _twist(self):
        """(linear, angular), zero once /odom has been silent for lost_after_s."""
        at = self._odom_at
        if at is None or time.monotonic() - at > self.lost_after:
            return 0.0, 0.0
        return self._linear, self._angular

    def states(self):
        if self._lost:
            return ["LOST"]
        linear = self._twist()[0]
        if abs(linear) > 0.01:
            return ["FORWARD" if linear > 0 else "REVERSE"]
        return []

    def _on_odom(self, msg):
        self._linear = msg.twist.twist.linear.x
        self._angular = msg.twist.twist.angular.z
        self._odom_at = time.monotonic()
        try:
            self.node.status.refresh()
        except Exception:
            log.exception("status refresh failed")

    def _tick(self):
        try:
            tf = self.node.tf_buffer.lookup_transform(self.node.map_frame, self.node.base_frame, Time())
            stamp = Time.from_msg(tf.header.stamp)
            # a static transform has a zero stamp: always fresh
            self._lost = stamp.nanoseconds != 0 and \
                self.node.get_clock().now() - stamp > Duration(seconds=self.lost_after)
        except Exception:  # TransformException, or no data yet
            self._lost, tf = True, None
        try:
            if not self._lost:
                t, q = tf.transform.translation, tf.transform.rotation
                linear, angular = self._twist()
                self.pub.publish_odometry(odometry_body(
                    self.node.ccs, t.x, t.y, yaw_from_quaternion(q), linear, angular))
        except Exception:
            log.exception("odometry publish failed")
        try:
            self.node.status.refresh()
        except Exception:
            log.exception("status refresh failed")

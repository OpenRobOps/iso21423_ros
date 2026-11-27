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

import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from tf2_ros import Buffer

from iso21423_ros.ccs import CcsFrame
from iso21423_ros.sources import paths
from iso21423_ros.sources.battery import BatterySource
from iso21423_ros.sources.odometry import OdometrySource
from iso21423_ros.sources.paths import GlobalPlanSource, LocalTrajectorySource
from iso21423_ros.status import StatusComposer


class FakePub:
    def __init__(self):
        self.calls = []
        self.fail = False

    def _rec(self, name, arg):
        if self.fail:
            raise OSError("broker down")
        self.calls.append((name, arg))

    def publish_odometry(self, body): self._rec("odometry", body)
    def publish_battery_status(self, body): self._rec("battery", body)
    def publish_global_plan(self, gp): self._rec("global", gp)
    def publish_local_trajectory(self, pts): self._rec("local", pts)

    def of(self, name):
        return [a for n, a in self.calls if n == name]


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    n = Node("test_sources")
    n.ccs = CcsFrame("ccs", [])
    n.map_frame, n.base_frame = "map", "base_link"
    n.tf_buffer = Buffer()
    n.status_out = []
    n.status = StatusComposer(n.status_out.append, "MODE_AUTO")
    yield n
    n.destroy_node()


def set_tf(node, x, y, age_s=0.0):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = "map", "base_link"
    t.header.stamp = (node.get_clock().now() - rclpy.duration.Duration(seconds=age_s)).to_msg()
    t.transform.translation.x, t.transform.translation.y = x, y
    t.transform.rotation.w = 1.0
    node.tf_buffer.set_transform(t, "test")


def path(*xy, sec=0):
    p = Path()
    for x, y in xy:
        ps = PoseStamped()
        ps.header.stamp.sec = sec
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        p.poses.append(ps)
    return p


def test_lost_until_tf_available(node):
    pub = FakePub()
    src = OdometrySource(node, pub, {})
    node.status.add_contributor(src.states)
    src._tick()
    assert src.states() == ["LOST"] and pub.calls == []
    assert node.status_out[-1] == ["MODE_AUTO", "LOST"]
    set_tf(node, 1.0, 2.0)
    src._tick()
    assert src.states() == []
    body = pub.of("odometry")[0]
    assert body["pose"]["locationPoint"]["x"] == pytest.approx(1.0)
    assert node.status_out[-1] == ["MODE_AUTO"]


def test_lost_when_tf_stale_and_direction(node):
    pub = FakePub()
    src = OdometrySource(node, pub, {"lost_after_s": 2.0})
    set_tf(node, 0.0, 0.0, age_s=5.0)
    src._tick()
    assert src.states() == ["LOST"]
    set_tf(node, 0.0, 0.0)
    odom = Odometry()
    odom.twist.twist.linear.x = -0.5
    src._on_odom(odom)
    src._tick()
    assert src.states() == ["REVERSE"]
    odom.twist.twist.linear.x = 0.5
    src._on_odom(odom)
    assert src.states() == ["FORWARD"]
    src._tick()
    assert pub.of("odometry")[-1]["velocity"]["linear"] == 0.5


def test_odometry_uses_ccs_frame(node):
    node.ccs = CcsFrame("ccs", [((0, 0), (10, 0)), ((1, 0), (10, 1)), ((0, 1), (9, 0))])  # rot 90 + shift
    pub = FakePub()
    src = OdometrySource(node, pub, {})
    set_tf(node, 1.0, 0.0)
    src._tick()
    lp = pub.of("odometry")[0]["pose"]["locationPoint"]
    assert (lp["x"], lp["y"]) == pytest.approx((10.0, 1.0))


def test_odometry_publish_failure_survives(node, logcap):
    pub = FakePub()
    pub.fail = True
    src = OdometrySource(node, pub, {})
    set_tf(node, 0.0, 0.0)
    src._tick()  # must not raise
    assert "broker down" in logcap.text


def test_battery_states_and_body(node):
    pub = FakePub()
    src = BatterySource(node, pub, {"low_threshold": 0.2})
    node.status.add_contributor(src.states)
    assert src.states() == []
    m = BatteryState()
    m.percentage, m.power_supply_status = 0.1, BatteryState.POWER_SUPPLY_STATUS_CHARGING
    src._on_msg(m)
    assert src.states() == ["CHARGING", "LOW_BATTERY"]
    assert pub.of("battery")[0]["batteryChargingState"] == "CHARGING"
    assert node.status_out[-1] == ["MODE_AUTO", "CHARGING", "LOW_BATTERY"]
    m.percentage, m.power_supply_status = 0.9, BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
    src._on_msg(m)
    assert src.states() == []
    pub.fail = True
    src._on_msg(m)  # must not raise


def test_global_plan_deduped(node):
    pub = FakePub()
    src = GlobalPlanSource(node, pub, {})
    src._on_msg(path((0, 0), (1, 1), sec=1))
    src._on_msg(path((0, 0), (1, 1), sec=2))
    assert len(pub.of("global")) == 1
    src._on_msg(path((0, 0), (2, 2)))
    assert len(pub.of("global")) == 2
    pub.fail = True
    src._on_msg(path((5, 5)))  # must not raise
    pub.fail = False
    src._on_msg(path((5, 5)))  # failed publish is retried
    assert len(pub.of("global")) == 3


def test_local_trajectory_throttled(node, monkeypatch):
    now = [100.0]
    monkeypatch.setattr(paths.time, "monotonic", lambda: now[0])
    pub = FakePub()
    src = LocalTrajectorySource(node, pub, {"rate_hz": 2.0})
    src._on_msg(path((0, 0)))
    now[0] += 0.1
    src._on_msg(path((1, 1)))
    assert len(pub.of("local")) == 1
    now[0] += 0.5
    src._on_msg(path((2, 2)))
    assert len(pub.of("local")) == 2
    pub.fail = True
    now[0] += 1
    src._on_msg(path((3, 3)))  # must not raise


def test_static_tf_not_lost(node):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = "map", "base_link"
    t.transform.rotation.w = 1.0  # zero stamp
    node.tf_buffer.set_transform_static(t, "test")
    pub = FakePub()
    src = OdometrySource(node, pub, {})
    src._tick()
    assert src.states() == [] and len(pub.of("odometry")) == 1


def test_stale_twist_cleared(node):
    pub = FakePub()
    src = OdometrySource(node, pub, {"lost_after_s": 2.0})
    set_tf(node, 0.0, 0.0)
    odom = Odometry()
    odom.twist.twist.linear.x = 0.5
    src._tick()
    src._on_odom(odom)
    assert src.states() == ["FORWARD"]
    src._odom_at -= 3.0  # /odom went silent
    assert src.states() == []
    src._tick()
    assert pub.of("odometry")[-1]["velocity"]["linear"] == 0.0


def test_bad_plan_msg_does_not_raise(node, logcap):
    GlobalPlanSource(node, FakePub(), {})._on_msg(object())  # plan_key blows up
    assert logcap.records


def test_rate_must_be_positive(node):
    for cls in (OdometrySource, LocalTrajectorySource):
        with pytest.raises(ValueError):
            cls(node, FakePub(), {"rate_hz": 0})


def test_wired_through_executor(node):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    pub = FakePub()
    src = OdometrySource(node, pub, {"rate_hz": 20.0})
    set_tf(node, 1.0, 0.0)
    p = node.create_publisher(Odometry, "/odom", qos_profile_sensor_data)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    msg = Odometry()
    msg.twist.twist.linear.x = 0.5
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and not (src.states() == ["FORWARD"] and pub.of("odometry")):
        p.publish(msg)
        set_tf(node, 1.0, 0.0)
        ex.spin_once(timeout_sec=0.05)
    ex.remove_node(node)
    assert src.states() == ["FORWARD"]
    assert pub.of("odometry")[-1]["velocity"]["linear"] == 0.5

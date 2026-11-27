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

import math

import pytest
from geometry_msgs.msg import PoseStamped
from iso21423 import now_timestamp
from iso21423.schema import validate_message
from nav_msgs.msg import Path
from sensor_msgs.msg import BatteryState

from iso21423_ros import mappings as m
from iso21423_ros.ccs import CcsFrame

ROT90 = [((0, 0), (5, -2)), ((1, 0), (5, -1)), ((0, 1), (4, -2))]


@pytest.mark.parametrize("status,state", [(1, "CHARGING"), (2, "DISCHARGING"), (3, "NOT_CHARGING"),
                                          (4, "FULL"), (0, "UNKNOWN")])
def test_battery_charging_states(status, state):
    msg = BatteryState(power_supply_status=status, percentage=0.5, voltage=12.0, current=1.0, temperature=30.0)
    assert m.battery_body(msg)["batteryChargingState"] == state


@pytest.mark.parametrize("health,iso", [(1, "HEALTHY"), (2, "OVERHEAT"), (3, "DEAD"), (4, "OVERVOLTAGE"),
                                        (5, "FAILURE"), (6, "COLD"), (0, "UNKNOWN")])
def test_battery_health(health, iso):
    msg = BatteryState(power_supply_health=health, percentage=0.5, voltage=12.0, current=1.0, temperature=30.0)
    assert m.battery_body(msg)["batteryHealth"] == iso


def test_battery_mapping_table():
    msg = BatteryState(percentage=1.7, voltage=math.nan, current=-2.0, temperature=math.nan)
    b = m.battery_body(msg)
    assert b["batterySoc"] == 1.0 and b["batteryCurrent"] == -2.0
    assert "batteryVoltage" not in b and "batteryTemperature" not in b
    assert m.battery_body(BatteryState(percentage=-1.0, voltage=1.0, current=1.0, temperature=1.0))["batterySoc"] == 0.0


def test_odometry_body_shape():
    body = m.odometry_body(CcsFrame("0b6f2a3e-1111-4222-8333-444455556666", ROT90), 2, 1, 0.0, 0.3, 0.1)
    assert body["pose"]["locationPoint"]["x"] == pytest.approx(4)
    assert body["pose"]["orientation"]["yaw"] == pytest.approx(math.pi / 2)
    assert validate_message("odometry", {**body, "timestamp": now_timestamp()}).ok


def _path(xy, sec=0):
    p = Path()
    for x, y in xy:
        ps = PoseStamped()
        ps.header.stamp.sec = sec
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        p.poses.append(ps)
    return p


def test_plan_key_ignores_stamps():
    assert m.plan_key(_path([(1.00001, 2)], sec=1)) == m.plan_key(_path([(1.0, 2.0)], sec=9))
    assert m.plan_key(_path([(1, 2)])) != m.plan_key(_path([(1, 2.01)]))


def test_stamped_points_share_timestamp():
    pts = m.stamped_points(CcsFrame("c", []), _path([(1, 2), (3, 4)]))
    assert pts[0]["timestamp"] == pts[1]["timestamp"] and pts[1]["locationPoint"]["x"] == 3


def test_nav_goal_pose_converts_ccs_to_map():
    g = m.nav_goal_pose(CcsFrame("c", ROT90), {"x": 4, "y": 0}, math.pi / 2, "map")
    assert g.header.frame_id == "map"
    assert (g.pose.position.x, g.pose.position.y) == pytest.approx((2, 1))
    assert m.yaw_from_quaternion(g.pose.orientation) == pytest.approx(0.0, abs=1e-9)
    assert m.quaternion_from_yaw(0.0) == (0.0, 0.0, 0.0, 1.0)
    assert m.distance((0, 0), (3, 4)) == 5

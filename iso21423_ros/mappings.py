"""Pure ROS 2 message -> ISO 21423 body mappings. No I/O."""
import json
import math

from geometry_msgs.msg import PoseStamped
from iso21423 import now_timestamp

_CHARGING = {1: "CHARGING", 2: "DISCHARGING", 3: "NOT_CHARGING", 4: "FULL"}
_HEALTH = {1: "HEALTHY", 2: "OVERHEAT", 3: "DEAD", 4: "OVERVOLTAGE", 5: "FAILURE", 6: "COLD"}


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def odometry_body(frame, x, y, yaw, linear, angular):
    """x, y, yaw are in the map frame."""
    return {
        "pose": {"locationPoint": frame.location_point(x, y),
                 "orientation": {"yaw": frame.yaw_to_ccs(yaw), "pitch": 0, "roll": 0}},
        "velocity": {"linear": linear, "angular": angular},
    }


def battery_body(msg):
    pct = msg.percentage
    out = {"batterySoc": min(1.0, max(0.0, pct)) if math.isfinite(pct) else 0.0,
           "batteryChargingState": _CHARGING.get(msg.power_supply_status, "UNKNOWN"),
           "batteryHealth": _HEALTH.get(msg.power_supply_health, "UNKNOWN")}
    for key, val in (("batteryVoltage", msg.voltage), ("batteryCurrent", msg.current),
                     ("batteryTemperature", msg.temperature)):
        if math.isfinite(val):
            out[key] = val
    return out


def stamped_points(frame, path_msg):
    """nav2 leaves per-pose stamps at 0, so all points share `now`."""
    ts = now_timestamp()
    return [{"timestamp": ts, "locationPoint": frame.location_point(p.pose.position.x, p.pose.position.y)}
            for p in path_msg.poses]


def plan_key(path_msg):
    """Dedupe key: rounded positions only (nav2 restamps identical plans)."""
    return json.dumps([[round(float(p.pose.position.x), 3), round(float(p.pose.position.y), 3)] for p in path_msg.poses])


def nav_goal_pose(frame, location, yaw, map_frame):
    x, y = frame.to_map(location["x"], location["y"])
    msg = PoseStamped()
    msg.header.frame_id = map_frame
    msg.pose.position.x, msg.pose.position.y = float(x), float(y)
    q = quaternion_from_yaw(frame.yaw_to_map(yaw or 0.0))
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, q)
    return msg


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

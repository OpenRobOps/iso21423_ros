"""dock / undock: nav2 docking server (DockRobot / UndockRobot) when available, else navigate to the dock.

The docking actions exist on Jazzy only, so they are imported lazily. Goal handling mirrors nav.py
(own goal handle only, Event waits woken by future callbacks, every wait bounded or woken by
ctx.cancelled); it is a small local copy because NavRunner's loop is specific to NavigateToPose
pause/resume/tolerance.
"""
import logging
import threading
import time

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from iso21423_ros.mappings import nav_goal_pose

log = logging.getLogger(__name__)

_POLL_S = 0.05


def _import_actions():
    """(DockRobot, UndockRobot), or None where nav2_msgs lacks them (Humble)."""
    try:
        from nav2_msgs.action import DockRobot, UndockRobot
    except ImportError:
        return None
    return DockRobot, UndockRobot


def _run_goal(client, goal, ctx, timeout, label, server="docking server"):
    """Send goal, wait for its result, map it to an ActionResult. Cancels only its own handle."""
    wake = threading.Event()

    def canceled():
        return ctx.aborted("REJECTED", "canceled")

    def wait(fut, deadline):
        while not fut.done() and time.monotonic() < deadline:
            wake.wait(_POLL_S)
            wake.clear()
        return fut.done()

    deadline = time.monotonic() + timeout
    while not client.server_is_ready():
        if ctx.cancelled.is_set():
            return canceled()
        if time.monotonic() >= deadline:
            return ctx.aborted("GENERAL_FAILURE", f"{server} unavailable")
        ctx.cancelled.wait(_POLL_S)

    send = client.send_goal_async(goal)
    send.add_done_callback(lambda _: wake.set())
    if not wait(send, time.monotonic() + timeout):
        send.add_done_callback(_cancel_if_accepted)  # don't leak a goal accepted after we gave up
        return canceled() if ctx.cancelled.is_set() else \
            ctx.aborted("GENERAL_FAILURE", f"{label} goal response timeout")
    handle = send.result()  # OUR handle
    if not handle.accepted:
        return ctx.aborted("GENERAL_FAILURE", f"{label} goal rejected")
    result = handle.get_result_async()
    result.add_done_callback(lambda _: wake.set())

    lost_since = None
    while not result.done():
        if ctx.cancelled.is_set():
            _cancel_quietly(handle)
            wait(result, time.monotonic() + timeout)  # bounded wait for the server to finish it
            return canceled()
        if client.server_is_ready():
            lost_since = None
        elif lost_since is None:
            lost_since = time.monotonic()
        elif time.monotonic() - lost_since > timeout:
            _cancel_quietly(handle)
            return ctx.aborted("GENERAL_FAILURE", f"{server} lost")
        wake.wait(_POLL_S)
        wake.clear()
    if ctx.cancelled.is_set():
        return canceled()
    res = result.result()
    if res.status == GoalStatus.STATUS_CANCELED:
        return ctx.aborted("GENERAL_FAILURE", "goal canceled externally")
    if res.status == GoalStatus.STATUS_SUCCEEDED and res.result.success:
        return ctx.succeeded()
    return ctx.aborted("GENERAL_FAILURE", f"{label} error {res.result.error_code}: {res.result.error_msg}")


def _cancel_quietly(handle):
    try:
        handle.cancel_goal_async()
    except Exception:
        log.exception("cancel of docking goal failed")


def _cancel_if_accepted(future):
    try:
        handle = future.result()
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()
    except Exception:
        log.exception("cancel of late-accepted docking goal failed")


def _check_params(params, prefix, keys):
    out = {}
    for key, default in keys.items():
        v = params.get(key, default)
        number = isinstance(v, (int, float)) and not isinstance(v, bool)
        if not (number if isinstance(default, float) else isinstance(v, type(default))):
            raise ValueError(f"handlers.{prefix}.{key}: expected {type(default).__name__}, got {v!r}")
        out[key] = v
    return out


class DockHandler:
    def __init__(self, node, params):
        self.node = node
        p = _check_params(params, "dock", {"mode": "auto", "action": "/dock_robot", "dock_type": "",
                                           "server_timeout_s": 2.0})
        if p["mode"] not in ("auto", "docking_server", "navigate"):
            raise ValueError(f"handlers.dock.mode: expected auto|docking_server|navigate, got {p['mode']!r}")
        self.mode, self.dock_type = p["mode"], p["dock_type"]
        self.server_timeout_s = float(p["server_timeout_s"])
        self._active = 0
        self._lock = threading.Lock()  # leaf lock: guards _active updates only
        self._actions = _import_actions() if self.mode != "navigate" else None
        self._client = None
        if self.mode == "docking_server" and not self._actions:
            log.warning("handlers.dock.mode is docking_server but nav2_msgs DockRobot is not available")
        if self._actions:
            self._client = ActionClient(node, self._actions[0], p["action"], callback_group=ReentrantCallbackGroup())

    def states(self):
        """Lock-free: the composer calls this under its own lock."""
        return ["DOCKING"] if self._active else []

    def _refresh(self):
        try:
            self.node.status.refresh()
        except Exception:
            log.exception("status refresh failed")

    def execute(self, action, ctx):
        with self._lock:
            self._active += 1
        self._refresh()
        try:
            return self._execute(action["properties"], ctx)
        finally:
            with self._lock:
                self._active -= 1
            self._refresh()

    def _server_ready(self, ctx):
        if self._client is None:
            return False
        if self.mode == "docking_server":
            return True  # _run_goal bounds the wait and reports "server unavailable"
        deadline = time.monotonic() + self.server_timeout_s
        while not self._client.server_is_ready():
            if ctx.cancelled.is_set() or time.monotonic() >= deadline:
                return False
            ctx.cancelled.wait(_POLL_S)
        return True

    def _execute(self, props, ctx):
        loc = props.get("dockLocation")
        dock_id = props.get("dockId")
        by_id = bool(dock_id)
        wants_server = by_id or self.dock_type
        if wants_server and self.mode == "docking_server" and self._client is None:
            return ctx.aborted("GENERAL_FAILURE", "docking server unavailable")
        if wants_server and self._server_ready(ctx):
            if ctx.cancelled.is_set():
                return ctx.aborted("REJECTED", "canceled")
            goal = self._actions[0].Goal()
            if by_id:
                goal.use_dock_id, goal.dock_id = True, str(dock_id)
            else:
                if not loc:
                    return ctx.aborted("GENERAL_FAILURE", "dock: properties.dockLocation is required")
                goal.use_dock_id = False
                goal.dock_pose = nav_goal_pose(self.node.ccs, loc, None, self.node.map_frame)
                goal.dock_type = self.dock_type
            return _run_goal(self._client, goal, ctx, self.server_timeout_s, "dock")
        if ctx.cancelled.is_set():  # e.g. cancelled during the auto-mode server probe
            return ctx.aborted("REJECTED", "canceled")
        if not loc:
            return ctx.aborted("GENERAL_FAILURE", "dock: properties.dockLocation is required")
        pose = nav_goal_pose(self.node.ccs, loc, None, self.node.map_frame)
        return self.node.nav.navigate(pose, ctx)


class UndockHandler:
    def __init__(self, node, params):
        self.node = node
        p = _check_params(params, "undock", {"action": "/undock_robot", "dock_type": "", "server_timeout_s": 2.0})
        self.dock_type, self.server_timeout_s = p["dock_type"], float(p["server_timeout_s"])
        actions = _import_actions()
        self._actions = actions
        self._client = None
        if actions:
            self._client = ActionClient(node, actions[1], p["action"], callback_group=ReentrantCallbackGroup())

    @staticmethod
    def available():
        return _import_actions() is not None

    def execute(self, action, ctx):
        if self._client is None:
            return ctx.aborted("GENERAL_FAILURE", "undock: UndockRobot is not available on this ROS distro")
        goal = self._actions[1].Goal()
        goal.dock_type = self.dock_type
        return _run_goal(self._client, goal, ctx, self.server_timeout_s, "undock", "undocking server")

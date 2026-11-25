import itertools
import threading
import time
from types import SimpleNamespace

import pytest
import rclpy
from iso21423 import ActionContext
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

try:
    from nav2_msgs.action import DockRobot, UndockRobot
except ImportError:  # Humble: skip the tests below, but still import cleanly
    DockRobot = UndockRobot = None

pytestmark = pytest.mark.skipif(DockRobot is None, reason="nav2_msgs docking actions not available (Humble)")

from iso21423_ros.ccs import CcsFrame
from iso21423_ros.handlers import dock as dock_mod
from iso21423_ros.handlers.dock import DockHandler, UndockHandler
from iso21423_ros.status import StatusComposer

_names = itertools.count()


class FakeDock:
    """DockRobot + UndockRobot servers. mode: succeed | fail | hang."""

    def __init__(self, node, dock_name, undock_name):
        self.mode = "succeed"
        self.goals, self.cancels = [], []
        self.release = threading.Event()
        cb = dict(goal_callback=lambda _: GoalResponse.ACCEPT, cancel_callback=self._cancel,
                  callback_group=ReentrantCallbackGroup())
        self.d = ActionServer(node, DockRobot, dock_name, execute_callback=self._exec(DockRobot), **cb)
        self.u = ActionServer(node, UndockRobot, undock_name, execute_callback=self._exec(UndockRobot), **cb)

    def _cancel(self, gh):
        self.cancels.append(bytes(gh.goal_id.uuid))
        return CancelResponse.ACCEPT

    def _exec(self, typ):
        def run(gh):
            self.goals.append(gh.request)
            res = typ.Result()
            if self.mode == "hang":
                while not gh.is_cancel_requested and not self.release.wait(0.05):
                    pass
                if gh.is_cancel_requested:
                    gh.canceled()
                else:
                    gh.abort()
                return res
            if self.mode == "fail":
                res.success, res.error_code, res.error_msg = False, 904, "no dock"
                gh.abort()
            else:
                gh.succeed()
            return res
        return run


class Ctx:
    def __init__(self):
        self.updates = []
        self.ctx = ActionContext({}, {}, None, threading.Event(),
                                 lambda status, properties=None: self.updates.append((status, properties)))


def run_bg(fn, *args):
    box = {}
    t = threading.Thread(target=lambda: box.setdefault("r", fn(*args)), daemon=True)
    t.start()
    return t, box


def wait_for(pred, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not pred():
        assert time.monotonic() < deadline, "timed out"
        time.sleep(0.02)


def dock_action(**props):
    return {"properties": {"dockLocation": {"x": 1, "y": 2}, **props}}


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def env(ros):
    n = next(_names)
    dock_name, undock_name = f"/dock_test_{n}", f"/undock_test_{n}"
    node, srv_node = Node("test_dock"), Node("fake_dock")
    node.ccs, node.map_frame = CcsFrame("ccs", []), "map"
    node.status_out = []
    node.status = StatusComposer(node.status_out.append, "MODE_AUTO")
    node.navigated = []
    node.nav = SimpleNamespace(navigate=lambda pose, ctx, **kw: node.navigated.append(pose) or ctx.succeeded())
    fake = FakeDock(srv_node, dock_name, undock_name)
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(node)
    ex.add_node(srv_node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    yield node, fake, dock_name, undock_name
    fake.release.set()
    ex.shutdown()
    spin.join(timeout=5)
    node.destroy_node()
    srv_node.destroy_node()


def test_dock_by_id_uses_docking_server(env):
    node, fake, dn, _ = env
    c = Ctx()
    r = DockHandler(node, {"action": dn}).execute(dock_action(dockId="dock-7"), c.ctx)
    assert r.outcome == "succeeded"
    assert len(fake.goals) == 1 and node.navigated == []
    g = fake.goals[0]
    assert g.use_dock_id is True and g.dock_id == "dock-7"


def test_dock_by_pose_needs_dock_type(env):
    node, fake, dn, _ = env
    c = Ctx()
    DockHandler(node, {"action": dn}).execute(dock_action(), c.ctx)
    assert len(node.navigated) == 1 and fake.goals == []  # no dock_type: navigate fallback
    assert (node.navigated[0].pose.position.x, node.navigated[0].pose.position.y) == (1.0, 2.0)

    c = Ctx()
    DockHandler(node, {"action": dn, "dock_type": "charger"}).execute(dock_action(), c.ctx)
    assert len(node.navigated) == 1 and len(fake.goals) == 1
    g = fake.goals[0]
    assert g.use_dock_id is False and g.dock_type == "charger"
    assert (g.dock_pose.pose.position.x, g.dock_pose.header.frame_id) == (1.0, "map")


def test_auto_falls_back_when_no_server(env):
    node, fake, _, _ = env
    c = Ctx()
    h = DockHandler(node, {"action": "/nobody_home", "server_timeout_s": 0.3})
    h.execute(dock_action(dockId="d"), c.ctx)
    assert len(node.navigated) == 1 and fake.goals == []


def test_docking_server_mode_without_server_aborts(env):
    node, _, _, _ = env
    c = Ctx()
    h = DockHandler(node, {"mode": "docking_server", "action": "/nobody_home", "server_timeout_s": 0.3})
    r = h.execute(dock_action(dockId="d"), c.ctx)
    assert r.outcome == "aborted" and "unavailable" in r.message
    assert node.navigated == []


def test_docking_state_while_executing(env):
    node, fake, dn, _ = env
    fake.mode = "hang"
    h = DockHandler(node, {"action": dn})
    node.status.add_contributor(h.states)
    assert "DOCKING" not in h.states()
    c = Ctx()
    t, box = run_bg(h.execute, dock_action(dockId="d"), c.ctx)
    wait_for(lambda: h.states() == ["DOCKING"])
    wait_for(lambda: any("DOCKING" in s for s in node.status_out))
    fake.release.set()
    t.join(5)
    assert not t.is_alive() and h.states() == []
    assert "DOCKING" not in node.status_out[-1]


def test_cancel_cancels_own_goal(env):
    node, fake, dn, _ = env
    fake.mode = "hang"
    c = Ctx()
    t, box = run_bg(DockHandler(node, {"action": dn}).execute, dock_action(dockId="d"), c.ctx)
    wait_for(lambda: fake.goals)
    c.ctx.cancelled.set()
    t.join(5)
    assert not t.is_alive() and len(fake.cancels) == 1
    assert (box["r"].reason, box["r"].message) == ("REJECTED", "canceled")


def test_dock_failure_maps_error(env):
    node, fake, dn, _ = env
    fake.mode = "fail"
    c = Ctx()
    r = DockHandler(node, {"action": dn}).execute(dock_action(dockId="d"), c.ctx)
    assert (r.reason, r.message) == ("GENERAL_FAILURE", "dock error 904: no dock")


def test_undock(env):
    node, fake, _, un = env
    assert UndockHandler.available()
    c = Ctx()
    r = UndockHandler(node, {"action": un, "dock_type": "charger"}).execute({"properties": {}}, c.ctx)
    assert r.outcome == "succeeded" and fake.goals[0].dock_type == "charger"
    c = Ctx()
    r = UndockHandler(node, {"action": "/nobody_undock", "server_timeout_s": 0.3}).execute({"properties": {}}, c.ctx)
    assert r.outcome == "aborted" and "unavailable" in r.message


def test_undock_unavailable_when_import_fails(env, monkeypatch):
    node, _, _, _ = env
    monkeypatch.setattr(dock_mod, "_import_actions", lambda: None)
    assert not UndockHandler.available()
    c = Ctx()
    assert UndockHandler(node, {}).execute({"properties": {}}, c.ctx).outcome == "aborted"
    # dock degrades to navigate
    DockHandler(node, {}).execute(dock_action(dockId="d"), c.ctx)
    assert len(node.navigated) == 1

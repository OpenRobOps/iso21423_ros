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

import itertools
import threading
import time
from types import SimpleNamespace

import pytest
import rclpy
from iso21423 import ActionContext
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

try:
    from nav2_msgs.action import DockRobot, UndockRobot
except ImportError:  # Humble: skip the tests below, but still import cleanly
    DockRobot = UndockRobot = None

pytestmark = pytest.mark.skipif(DockRobot is None, reason="nav2_msgs docking actions not available (Humble)")

from conftest import Spinner
from iso21423_ros.ccs import CcsFrame
from iso21423_ros.handlers import dock as dock_mod
from iso21423_ros.handlers.dock import DockHandler, UndockHandler
from iso21423_ros.nav import NavRunner
from iso21423_ros.node import ConnectorNode
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


def _h(node, handler):
    node.handlers.append(handler)
    return handler


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
    node.handlers = []
    node.nav = SimpleNamespace(paused=False,
                               navigate=lambda pose, ctx, **kw: node.navigated.append(pose) or ctx.succeeded())
    fake = FakeDock(srv_node, dock_name, undock_name)
    spinner = Spinner(node, srv_node, num_threads=8)
    yield node, fake, dock_name, undock_name
    fake.release.set()
    for h in node.handlers:
        if h._client is not None:
            h._client.destroy()
    spinner.stop()
    node.destroy_node()
    srv_node.destroy_node()


def test_dock_by_id_uses_docking_server(env):
    node, fake, dn, _ = env
    c = Ctx()
    r = _h(node, DockHandler(node, {"action": dn})).execute(dock_action(dockId="dock-7"), c.ctx)
    assert r.outcome == "succeeded"
    assert len(fake.goals) == 1 and node.navigated == []
    g = fake.goals[0]
    assert g.use_dock_id is True and g.dock_id == "dock-7"


def test_dock_by_pose_needs_dock_type(env):
    node, fake, dn, _ = env
    c = Ctx()
    _h(node, DockHandler(node, {"action": dn})).execute(dock_action(), c.ctx)
    assert len(node.navigated) == 1 and fake.goals == []  # no dock_type: navigate fallback
    assert (node.navigated[0].pose.position.x, node.navigated[0].pose.position.y) == (1.0, 2.0)

    c = Ctx()
    _h(node, DockHandler(node, {"action": dn, "dock_type": "charger"})).execute(dock_action(), c.ctx)
    assert len(node.navigated) == 1 and len(fake.goals) == 1
    g = fake.goals[0]
    assert g.use_dock_id is False and g.dock_type == "charger"
    assert (g.dock_pose.pose.position.x, g.dock_pose.header.frame_id) == (1.0, "map")


def test_auto_falls_back_when_no_server(env):
    node, fake, _, _ = env
    c = Ctx()
    h = _h(node, DockHandler(node, {"action": "/nobody_home", "server_timeout_s": 0.3}))
    h.execute(dock_action(dockId="d"), c.ctx)
    assert len(node.navigated) == 1 and fake.goals == []


def test_docking_server_mode_without_server_aborts(env):
    node, _, _, _ = env
    c = Ctx()
    h = _h(node, DockHandler(node, {"mode": "docking_server", "action": "/nobody_home", "server_timeout_s": 0.3}))
    r = h.execute(dock_action(dockId="d"), c.ctx)
    assert r.outcome == "aborted" and "unavailable" in r.message
    assert node.navigated == []


def test_docking_state_while_executing(env):
    node, fake, dn, _ = env
    fake.mode = "hang"
    h = _h(node, DockHandler(node, {"action": dn}))
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
    t, box = run_bg(_h(node, DockHandler(node, {"action": dn})).execute, dock_action(dockId="d"), c.ctx)
    wait_for(lambda: fake.goals)
    c.ctx.cancelled.set()
    t.join(5)
    assert not t.is_alive() and len(fake.cancels) == 1
    assert (box["r"].reason, box["r"].message) == ("REJECTED", "canceled")


def test_dock_failure_maps_error(env):
    node, fake, dn, _ = env
    fake.mode = "fail"
    c = Ctx()
    r = _h(node, DockHandler(node, {"action": dn})).execute(dock_action(dockId="d"), c.ctx)
    assert (r.reason, r.message) == ("GENERAL_FAILURE", "dock error 904: no dock")


def test_undock(env):
    node, fake, _, un = env
    assert UndockHandler.available()
    c = Ctx()
    r = _h(node, UndockHandler(node, {"action": un, "dock_type": "charger"})).execute({"properties": {}}, c.ctx)
    assert r.outcome == "succeeded" and fake.goals[0].dock_type == "charger"
    c = Ctx()
    r = _h(node, UndockHandler(node, {"action": "/nobody_undock", "server_timeout_s": 0.3})).execute({"properties": {}}, c.ctx)
    assert r.outcome == "aborted" and "unavailable" in r.message


def test_undock_unavailable_when_import_fails(env, monkeypatch):
    node, _, _, _ = env
    monkeypatch.setattr(dock_mod, "_import_actions", lambda: None)
    assert not UndockHandler.available()
    c = Ctx()
    assert _h(node, UndockHandler(node, {})).execute({"properties": {}}, c.ctx).outcome == "aborted"
    # dock degrades to navigate
    _h(node, DockHandler(node, {})).execute(dock_action(dockId="d"), c.ctx)
    assert len(node.navigated) == 1


def test_cancel_during_server_probe_does_not_navigate(env):
    node, _, _, _ = env
    c = Ctx()
    c.ctx.cancelled.set()
    h = DockHandler(node, {"action": "/nobody_home", "server_timeout_s": 5.0})
    node.handlers.append(h)
    r = h.execute(dock_action(dockId="d"), c.ctx)
    assert (r.reason, r.message) == ("REJECTED", "canceled") and node.navigated == []


def test_docking_server_mode_import_failure_aborts(env, monkeypatch):
    node, _, _, _ = env
    monkeypatch.setattr(dock_mod, "_import_actions", lambda: None)
    r = _h(node, DockHandler(node, {"mode": "docking_server"})).execute(dock_action(dockId="d"), Ctx().ctx)
    assert (r.reason, r.message) == ("GENERAL_FAILURE", "docking server unavailable") and node.navigated == []


def test_bool_float_param_rejected(env):
    node, _, _, _ = env
    with pytest.raises(ValueError):
        DockHandler(node, {"server_timeout_s": True})


def test_navigate_mode_ignores_server(env):
    node, fake, dn, _ = env
    h = DockHandler(node, {"mode": "navigate", "action": dn, "dock_type": "charger"})
    node.handlers.append(h)
    h.execute(dock_action(dockId="d"), Ctx().ctx)
    assert len(node.navigated) == 1 and fake.goals == [] and h._client is None


def test_server_loss_mid_dock_aborts(env):
    node, fake, dn, _ = env
    fake.mode = "hang"
    h = DockHandler(node, {"action": dn, "server_timeout_s": 0.5})
    node.handlers.append(h)
    c = Ctx()
    t, box = run_bg(h.execute, dock_action(dockId="d"), c.ctx)
    wait_for(lambda: fake.goals)
    fake.d.destroy()  # the server goes away
    t.join(5)
    assert not t.is_alive() and box["r"].message == "docking server lost"


def test_exception_clears_docking(env):
    node, _, _, _ = env
    h = DockHandler(node, {"mode": "navigate"})
    node.handlers.append(h)
    node.nav = SimpleNamespace(navigate=lambda *a, **k: 1 / 0)
    with pytest.raises(ZeroDivisionError):
        h.execute(dock_action(), Ctx().ctx)
    assert h.states() == []


def test_dock_shows_docking_never_idle(env):
    """Through the node's request tracking, with the nav contributor too: IDLE needs no request running."""
    node, fake, dn, _ = env
    fake.mode = "hang"
    node._inflight, node._idle = 0, threading.Condition()
    node.nav = NavRunner(node, "/unused_nav")
    h = _h(node, DockHandler(node, {"action": dn}))
    node.status.add_contributor(h.states)
    node.status.add_contributor(node.nav.states)
    node.status.refresh()
    assert node.status_out[-1] == ["MODE_AUTO", "IDLE"]
    start = len(node.status_out)
    t, box = run_bg(ConnectorNode._tracked(node, h.execute), dock_action(dockId="d"), Ctx().ctx)
    wait_for(lambda: fake.goals)
    assert node.status_out[-1] == ["MODE_AUTO", "DOCKING"]
    fake.release.set()
    t.join(5)
    assert box["r"].outcome == "aborted"  # hang + release = server abort
    assert not any("IDLE" in s for s in node.status_out[start:-1])
    assert node.status_out[-1] == ["MODE_AUTO", "IDLE"]


@pytest.mark.parametrize("cls", [DockHandler, UndockHandler])
def test_pause_cancels_dock_goal_and_resume_resends(env, cls):
    node, fake, dn, un = env
    fake.mode = "hang"
    h = _h(node, cls(node, {"action": dn if cls is DockHandler else un}))
    c = Ctx()
    t, box = run_bg(h.execute, dock_action(dockId="d"), c.ctx)
    wait_for(lambda: fake.goals)
    node.nav.paused = True
    wait_for(lambda: len(fake.cancels) == 1)
    time.sleep(0.3)
    assert len(fake.goals) == 1 and t.is_alive()  # held while paused, no new goal
    fake.mode = "succeed"
    node.nav.paused = False
    t.join(5)
    assert not t.is_alive() and box["r"].outcome == "succeeded"
    assert len(fake.goals) == 2 and len(fake.cancels) == 1

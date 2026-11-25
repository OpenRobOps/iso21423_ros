import itertools
import threading
import time

import pytest
import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from iso21423 import ActionContext
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from iso21423_ros.ccs import CcsFrame
from iso21423_ros.handlers.move import MoveHandler
from iso21423_ros.handlers.pause import PauseHandler, ResumeHandler
from iso21423_ros.nav import NavRunner
from iso21423_ros.status import StatusComposer

_names = itertools.count()


class FakeNav:
    """In-process NavigateToPose server. Goals drive current_pose from (0,0) to the goal in `duration`."""

    def __init__(self, node, name):
        self.duration, self.mode = 1.0, "succeed"  # mode: succeed | abort | reject | hang
        self.single_goal = False  # like real nav2: a new goal aborts the running one
        self.on_preempted = None  # called just before a single_goal preemption aborts a goal
        self.release = threading.Event()  # ends "hang" goals
        self.goals, self.cancels = [], []  # goal uuids (bytes) in arrival order
        self.server = ActionServer(node, NavigateToPose, name, execute_callback=self._exec,
                                   goal_callback=self._goal, cancel_callback=self._cancel,
                                   callback_group=ReentrantCallbackGroup())

    def _goal(self, _req):
        return GoalResponse.REJECT if self.mode == "reject" else GoalResponse.ACCEPT

    def _cancel(self, gh):
        self.cancels.append(bytes(gh.goal_id.uuid))
        return CancelResponse.ACCEPT

    def _exec(self, gh):
        gid = bytes(gh.goal_id.uuid)
        self.goals.append(gid)
        if self.mode == "hang":  # accepted, then silence: no feedback, no result
            self.release.wait(10)
            gh.abort()
            return NavigateToPose.Result()
        tx, ty = gh.request.pose.pose.position.x, gh.request.pose.pose.position.y
        start = time.monotonic()
        while (f := (time.monotonic() - start) / self.duration) < 1.0:
            if gh.is_cancel_requested:
                gh.canceled()
                return NavigateToPose.Result()
            if self.single_goal and self.goals[-1] != gid:
                if self.on_preempted:
                    self.on_preempted()
                gh.abort()
                res = NavigateToPose.Result()
                res.error_code, res.error_msg = 100, "preempted"
                return res
            fb = NavigateToPose.Feedback()
            fb.current_pose.pose.position.x, fb.current_pose.pose.position.y = tx * f, ty * f
            fb.distance_remaining = float((1 - f) * (tx * tx + ty * ty) ** 0.5)
            gh.publish_feedback(fb)
            time.sleep(0.05)
        res = NavigateToPose.Result()
        if self.mode == "abort":
            if hasattr(res, "error_code"):  # absent from NavigateToPose on Humble
                res.error_code, res.error_msg = 104, "no valid path"
            gh.abort()
        else:
            gh.succeed()
        return res


class Ctx:
    """Real library ActionContext plus a record of the updates it sends."""

    def __init__(self):
        self.updates = []
        self.ctx = ActionContext({}, {}, None, threading.Event(),
                                 lambda status, properties=None: self.updates.append((status, properties)))

    @property
    def progress(self):
        return [p for s, p in self.updates if s == "EXECUTING"]


def wait_for(pred, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not pred():
        assert time.monotonic() < deadline, "timed out"
        time.sleep(0.02)


def pose(x, y):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.w = 1.0
    return p


def run_bg(fn, *args, **kw):
    box = {}
    t = threading.Thread(target=lambda: box.setdefault("r", fn(*args, **kw)), daemon=True)
    t.start()
    return t, box


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def env(ros):
    """(node, fake server, runner factory) with a MultiThreadedExecutor spinning in the background."""
    name = f"/nav_test_{next(_names)}"
    node, srv_node = Node("test_nav"), Node("fake_nav")
    node.ccs, node.map_frame = CcsFrame("ccs", []), "map"
    node.status_out = []
    node.status = StatusComposer(node.status_out.append, "MODE_AUTO")
    fake = FakeNav(srv_node, name)
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(node)
    ex.add_node(srv_node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    def make(**kw):
        kw.setdefault("action_name", name)
        kw.setdefault("server_timeout_s", 2.0)
        kw.setdefault("progress_min_interval_s", 0.3)
        node.nav = NavRunner(node, **kw)
        node.status.add_contributor(node.nav.states)
        return node.nav

    yield node, fake, make
    fake.release.set()
    ex.shutdown()
    spin.join(timeout=5)
    node.destroy_node()
    srv_node.destroy_node()


def test_move_succeeds_and_reports_progress(env):
    node, fake, make = env
    make()
    c = Ctx()
    r = MoveHandler(node, {}).execute(
        {"type": "move", "properties": {"location": {"x": 3, "y": 4}, "orientation": {"yaw": 0}}}, c.ctx)
    assert r.outcome == "succeeded"
    assert len(fake.goals) == 1 and fake.cancels == []
    assert 2 <= len(c.progress) <= 5  # 1 s goal, 0.3 s throttle
    assert all(set(p) == {"distanceRemaining"} for p in c.progress)
    assert c.progress[0]["distanceRemaining"] > c.progress[-1]["distanceRemaining"]
    assert node.nav.states() == ["IDLE"]


def test_cancel_cancels_own_goal_and_aborts_rejected(env):
    _, fake, make = env
    nav = make()
    fake.duration = 10.0
    c = Ctx()
    t, box = run_bg(nav.navigate, pose(1, 0), c.ctx)
    wait_for(lambda: fake.goals)
    assert nav.states() == []  # busy
    c.ctx.cancelled.set()
    t.join(timeout=3)
    assert box["r"].outcome == "aborted" and box["r"].reason == "REJECTED"
    assert fake.cancels == fake.goals


def test_pause_resume_keeps_move_executing(env):
    node, fake, make = env
    nav = make()
    fake.duration = 30.0  # the pause always lands while goal 1 is running
    c = Ctx()
    t, box = run_bg(nav.navigate, pose(1, 0), c.ctx)
    wait_for(lambda: fake.goals)
    before_pause = len(node.status_out)
    assert PauseHandler(node, {}).execute({}, c.ctx).outcome == "succeeded"
    wait_for(lambda: fake.cancels == fake.goals)
    assert nav.paused and nav.states() == ["PAUSED"]
    time.sleep(0.5)
    assert len(fake.goals) == 1 and t.is_alive()  # objective held, no new goal while paused
    fake.duration = 0.5
    before_resume = len(node.status_out)
    assert ResumeHandler(node, {}).execute({}, c.ctx).outcome == "succeeded"
    assert nav.states() == []
    t.join(timeout=5)
    assert box["r"].outcome == "succeeded"
    assert len(fake.goals) == 2 and fake.cancels == fake.goals[:1]
    assert nav.states() == ["IDLE"] and node.status_out[-1] == ["MODE_AUTO", "IDLE"]
    assert all(s == "EXECUTING" for s, _ in c.updates)
    # PAUSED is published exactly while paused
    out = node.status_out
    assert before_resume > before_pause
    assert all("PAUSED" in st for st in out[before_pause:before_resume])
    assert not any("PAUSED" in st for st in out[:before_pause] + out[before_resume:])


def test_preempted_move_does_not_cancel_successor_goal(env):
    _, fake, make = env
    nav = make()
    fake.duration = 1.5
    a, b = Ctx(), Ctx()
    ta, box_a = run_bg(nav.navigate, pose(1, 0), a.ctx)
    wait_for(lambda: len(fake.goals) == 1)
    tb, box_b = run_bg(nav.navigate, pose(2, 0), b.ctx)
    wait_for(lambda: len(fake.goals) == 2)  # B accepted before A observes its cancel
    a.ctx.cancelled.set()
    ta.join(timeout=3)
    assert box_a["r"].reason == "REJECTED"
    tb.join(timeout=5)
    assert box_b["r"].outcome == "succeeded"
    goal_a, goal_b = fake.goals
    assert fake.cancels == [goal_a] and goal_b not in fake.cancels


def test_move_aborts_when_server_unavailable(env):
    _, _, make = env
    nav = make(action_name="/no_such_server", server_timeout_s=0.5)
    start = time.monotonic()
    r = nav.navigate(pose(1, 0), Ctx().ctx)
    assert time.monotonic() - start < 2.0
    assert (r.outcome, r.reason, r.message) == ("aborted", "GENERAL_FAILURE", "navigate_to_pose server unavailable")
    assert nav.states() == ["IDLE"]


def test_goal_rejected_aborts(env):
    _, fake, make = env
    nav = make()
    fake.mode = "reject"
    r = nav.navigate(pose(1, 0), Ctx().ctx)
    assert (r.outcome, r.reason, r.message) == ("aborted", "GENERAL_FAILURE", "goal rejected")


def test_nav2_abort_maps_error(env):
    _, fake, make = env
    nav = make()
    fake.duration, fake.mode = 0.2, "abort"
    r = nav.navigate(pose(1, 0), Ctx().ctx)
    want = "nav2 error 104: no valid path" if hasattr(NavigateToPose.Result(), "error_code") else "nav2 error 0: "
    assert (r.outcome, r.reason, r.message) == ("aborted", "GENERAL_FAILURE", want)


def move(x, y, **props):
    return {"type": "move", "properties": {"location": {"x": x, "y": y}, **props}}


def test_tolerance_radius_finishes_early(env):
    node, fake, make = env
    make()
    fake.duration = 5.0
    start = time.monotonic()
    r = MoveHandler(node, {}).execute(move(10.0, 0.0, toleranceRadius=9), Ctx().ctx)
    assert r.outcome == "succeeded"
    assert time.monotonic() - start < 3.0
    assert fake.cancels == fake.goals  # stopped nav2 once inside the circle


@pytest.mark.parametrize("tol", [-1.0, "1", True, float("nan"), [1]])
def test_invalid_tolerance_radius_aborts(env, tol):
    node, fake, make = env
    make()
    r = MoveHandler(node, {}).execute(move(1.0, 0.0, toleranceRadius=tol), Ctx().ctx)
    assert (r.outcome, r.reason) == ("aborted", "GENERAL_FAILURE") and "toleranceRadius" in r.message
    assert fake.goals == []


def test_server_lost_mid_goal_aborts(env):
    _, fake, make = env
    nav = make(server_timeout_s=0.5)
    fake.mode = "hang"
    t, box = run_bg(nav.navigate, pose(1, 0), Ctx().ctx)
    wait_for(lambda: fake.goals)
    fake.server.destroy()
    start = time.monotonic()
    t.join(timeout=3)
    assert time.monotonic() - start < 2 * 0.5 + 1.0  # loss detection + timeout
    assert (box["r"].reason, box["r"].message) == ("GENERAL_FAILURE", "navigate_to_pose server lost")


def test_late_cancel_with_nav2_abort_reports_canceled(env):
    """Real nav2 is single-goal: B's goal makes nav2 abort A's goal, which A sees before its cancel poll."""
    _, fake, make = env
    nav = make()
    fake.duration, fake.single_goal = 1.0, True
    a = Ctx()
    fake.on_preempted = a.ctx.cancelled.set  # the library cancels A, and nav2 aborts A's goal right away
    ta, box_a = run_bg(nav.navigate, pose(1, 0), a.ctx)
    wait_for(lambda: len(fake.goals) == 1)
    tb, box_b = run_bg(nav.navigate, pose(2, 0), Ctx().ctx)
    ta.join(timeout=3)
    tb.join(timeout=5)
    assert (box_a["r"].reason, box_a["r"].message) == ("REJECTED", "canceled")
    assert box_b["r"].outcome == "succeeded"
    assert fake.goals[1] not in fake.cancels


def test_external_cancel_aborts_general_failure(env):
    node, fake, make = env
    nav = make()
    fake.duration = 10.0
    t, box = run_bg(nav.navigate, pose(1, 0), Ctx().ctx)
    wait_for(lambda: fake.goals)
    cli = node.create_client(CancelGoal, nav.action_name + "/_action/cancel_goal")
    assert cli.wait_for_service(timeout_sec=2)
    cli.call_async(CancelGoal.Request())  # zero id + zero stamp = cancel all
    t.join(timeout=3)
    assert (box["r"].reason, box["r"].message) == ("GENERAL_FAILURE", "goal canceled externally")


def test_idle_state_when_no_objective(env):
    node, _, make = env
    nav = make()
    assert nav.states() == ["IDLE"] and not nav.busy
    PauseHandler(node, {}).execute({}, Ctx().ctx)
    assert nav.states() == ["PAUSED"] and node.status_out[-1] == ["MODE_AUTO", "PAUSED"]
    ResumeHandler(node, {}).execute({}, Ctx().ctx)
    assert nav.states() == ["IDLE"] and node.status_out[-1] == ["MODE_AUTO", "IDLE"]


def test_raising_progress_does_not_kill_move(env, logcap):
    _, fake, make = env
    nav = make()
    fake.duration = 0.5
    c = Ctx()
    c.ctx._update = lambda *a, **k: (_ for _ in ()).throw(OSError("broker down"))
    assert nav.navigate(pose(1, 0), c.ctx).outcome == "succeeded"
    assert "broker down" in logcap.text

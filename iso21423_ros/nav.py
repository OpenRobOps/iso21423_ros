"""NavigateToPose runner shared by move/dock and pause/resume.

navigate() runs on a library worker thread and never spins the executor: it waits on a per-call
Event set by future done-callbacks and by pause()/resume(). Each call keeps its OWN goal handle and
only ever cancels that one, so a late-cancelled (preempted) call can't touch its successor's goal.
"""
import logging
import threading
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from iso21423_ros.mappings import distance

log = logging.getLogger(__name__)

_POLL_S = 0.05  # ctx.cancelled has no callback hook, so waits poll it at this period


class NavRunner:
    def __init__(self, node, action_name="/navigate_to_pose", server_timeout_s=5.0, progress_min_interval_s=2.0):
        self.node = node
        self.action_name = action_name
        self.server_timeout_s = float(server_timeout_s)
        self.progress_min_interval_s = float(progress_min_interval_s)
        self._client = ActionClient(node, NavigateToPose, action_name, callback_group=ReentrantCallbackGroup())
        self._paused = False
        self._busy = 0
        self._wakers = set()
        self._lock = threading.Lock()  # leaf lock: guards _busy and _wakers only

    @property
    def paused(self):
        return self._paused

    @property
    def busy(self):
        return self._busy > 0

    def states(self):
        """Lock-free: the composer calls this under its own lock."""
        if self._paused:
            return ["PAUSED"]
        return [] if self._busy else ["IDLE"]

    def pause(self):
        self._set_paused(True)

    def resume(self):
        self._set_paused(False)

    def _set_paused(self, value):
        self._paused = value
        with self._lock:
            wakers = list(self._wakers)
        for w in wakers:
            w.set()
        self._refresh()

    def _refresh(self):
        try:
            self.node.status.refresh()
        except Exception:
            log.exception("status refresh failed")

    def navigate(self, pose, ctx, tolerance_radius=None, target_xy=None):
        wake = threading.Event()
        with self._lock:
            self._busy += 1
            self._wakers.add(wake)
        self._refresh()
        try:
            return self._navigate(pose, ctx, tolerance_radius, target_xy, wake)
        finally:
            with self._lock:
                self._busy -= 1
                self._wakers.discard(wake)
            self._refresh()

    def _navigate(self, pose, ctx, tolerance_radius, target_xy, wake):
        if target_xy is None:
            target_xy = (pose.pose.position.x, pose.pose.position.y)

        def canceled():  # the executor maps REJECTED-after-cancel to CANCELED
            return ctx.aborted("REJECTED", "canceled")

        last_progress = [float("-inf")]
        while True:
            # 1. hold the objective while paused, no goal sent
            while self._paused:
                if ctx.cancelled.is_set():
                    return canceled()
                self._sleep(wake)
            if ctx.cancelled.is_set():
                return canceled()

            # 2. send our goal
            deadline = time.monotonic() + self.server_timeout_s
            while not self._client.server_is_ready():
                if ctx.cancelled.is_set():
                    return canceled()
                if time.monotonic() >= deadline:
                    return ctx.aborted("GENERAL_FAILURE", "navigate_to_pose server unavailable")
                ctx.cancelled.wait(_POLL_S)
            inside = threading.Event()
            done = threading.Event()  # this goal is over for us: drop late feedback

            def on_feedback(msg, inside=inside, done=done):
                if done.is_set():
                    return
                try:  # runs on the executor thread: never let it raise
                    fb = msg.feedback
                    now = time.monotonic()
                    if now - last_progress[0] >= self.progress_min_interval_s:
                        last_progress[0] = now
                        try:
                            ctx.progress({"distanceRemaining": float(fb.distance_remaining)})
                        except Exception:
                            log.exception("move progress publish failed")
                    if tolerance_radius is not None:
                        p = fb.current_pose.pose.position
                        if distance((p.x, p.y), target_xy) <= tolerance_radius:
                            inside.set()
                            wake.set()
                except Exception:
                    log.exception("navigate_to_pose feedback handling failed")

            goal = NavigateToPose.Goal()
            goal.pose = pose
            send = self._client.send_goal_async(goal, feedback_callback=on_feedback)
            send.add_done_callback(lambda _: wake.set())
            if not self._wait(send, wake, deadline=time.monotonic() + self.server_timeout_s):
                done.set()
                send.add_done_callback(_cancel_if_accepted)  # don't leak a goal accepted after we gave up
                return canceled() if ctx.cancelled.is_set() else \
                    ctx.aborted("GENERAL_FAILURE", "navigate_to_pose goal response timeout")
            handle = send.result()  # OUR handle; the only one this call may cancel
            if not handle.accepted:
                return ctx.aborted("GENERAL_FAILURE", "goal rejected")
            result = handle.get_result_async()
            result.add_done_callback(lambda _: wake.set())

            # 3. watch cancel / pause / tolerance until the result arrives
            while not result.done():
                if ctx.cancelled.is_set():
                    done.set()
                    self._cancel(handle, result, wake)
                    return canceled()
                if inside.is_set():
                    done.set()
                    self._cancel(handle, result, wake)
                    return ctx.succeeded()
                if self._paused:
                    done.set()
                    self._cancel(handle, result, wake)
                    if result.done() and result.result().status == GoalStatus.STATUS_SUCCEEDED:
                        return ctx.succeeded()  # arrived before the cancel landed
                    break  # back to step 1: wait for resume, then re-send
                self._sleep(wake)
            else:
                done.set()
                # 4. result not caused by us
                res = result.result()
                if res.status == GoalStatus.STATUS_SUCCEEDED:
                    return ctx.succeeded()
                if res.status == GoalStatus.STATUS_CANCELED:
                    return ctx.aborted("GENERAL_FAILURE", "goal canceled externally")
                code = getattr(res.result, "error_code", 0)
                msg = getattr(res.result, "error_msg", "")
                return ctx.aborted("GENERAL_FAILURE", f"nav2 error {code}: {msg}")

    def _sleep(self, wake):
        wake.wait(_POLL_S)
        wake.clear()  # callers re-check state after this, so a set() racing the clear is not lost

    def _wait(self, future, wake, deadline):
        while not future.done() and time.monotonic() < deadline:
            self._sleep(wake)
        return future.done()

    def _cancel(self, handle, result, wake):
        """Cancel our own goal and wait (bounded) for nav2 to finish it."""
        try:
            handle.cancel_goal_async()
        except Exception:
            log.exception("cancel of navigate_to_pose goal failed")
            return
        if not self._wait(result, wake, deadline=time.monotonic() + self.server_timeout_s):
            log.warning("navigate_to_pose goal did not finish within %.1fs of cancel", self.server_timeout_s)


def _cancel_if_accepted(future):
    try:
        handle = future.result()
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()
    except Exception:
        log.exception("cancel of late-accepted navigate_to_pose goal failed")

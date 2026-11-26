import logging
import threading

import pytest

from iso21423_ros.status import StatusComposer


def make(mode="MODE_AUTO"):
    out = []
    return StatusComposer(out.append, mode), out


def test_mode_first_and_priority_order():
    c, out = make()
    for s in (["IDLE"], ["CHARGING", "LOW_BATTERY"], ["LOST"]):
        c.add_contributor(lambda s=s: s)
    c.refresh()
    assert out[-1] == ["MODE_AUTO", "LOST", "CHARGING", "LOW_BATTERY", "IDLE"]
    assert c.states() == out[-1]


def test_publishes_only_on_change():
    c, out = make()
    cur = ["IDLE"]
    c.add_contributor(lambda: cur)
    c.refresh()
    c.refresh()
    assert len(out) == 1
    cur[:] = ["FORWARD"]
    c.refresh()
    assert len(out) == 2
    c.set_mode("MODE_TELEOP")
    assert out[-1][0] == "MODE_TELEOP"


def test_vendor_state_after_known():
    c, out = make()
    c.add_contributor(lambda: ["ZZZ", "IDLE", "AAA", "IDLE"])
    c.refresh()
    assert out[-1] == ["MODE_AUTO", "IDLE", "ZZZ", "AAA"]


def test_contributor_mode_ignored(logcap):
    c, out = make()
    c.add_contributor(lambda: ["MODE_MANUAL", "IDLE"])
    with logcap.at_level(logging.WARNING):
        c.refresh()
        c.refresh()
    assert out[-1] == ["MODE_AUTO", "IDLE"]
    assert len([r for r in logcap.records if "MODE_MANUAL" in r.getMessage()]) == 1


def test_never_emits_ready():
    c, out = make()
    c.refresh()
    assert out[-1] == ["MODE_AUTO"]
    assert "READY" not in out[-1] and "NOT_READY" not in out[-1]


def test_contributor_exception_skipped_and_logged(logcap):
    c, out = make()

    def bad():
        raise RuntimeError("boom")

    c.add_contributor(bad)
    c.add_contributor(lambda: ["IDLE"])
    with logcap.at_level(logging.ERROR):
        c.refresh()
    assert out[-1] == ["MODE_AUTO", "IDLE"]
    assert "boom" in logcap.text


def test_invalid_mode():
    with pytest.raises(ValueError):
        StatusComposer(lambda s: None, "MODE_BOGUS")
    c, _ = make()
    with pytest.raises(ValueError):
        c.set_mode("READY")
    assert c.states() == []
    c.refresh()
    with pytest.raises(ValueError):
        c.set_mode("READY")
    assert c.states() == ["MODE_AUTO"]


def test_publish_failure_retried_and_not_raised(logcap):
    calls = []

    def pub(states):
        calls.append(states)
        if len(calls) == 1:
            raise OSError("broker down")

    c = StatusComposer(pub, "MODE_AUTO")
    c.refresh()  # must not raise
    assert c.states() == []
    assert "broker down" in logcap.text
    c.refresh()
    assert len(calls) == 2 and c.states() == ["MODE_AUTO"]


def test_bad_contributor_output_skipped(logcap):
    c, out = make()
    c.add_contributor(lambda: [None])
    c.add_contributor(lambda: "IDLE")
    c.add_contributor(lambda: ("LOST",))
    c.refresh()
    assert out[-1] == ["MODE_AUTO", "LOST"]
    assert len(logcap.records) == 2


def test_concurrent_refresh_publishes_in_order():
    out = []
    inflight = [0, 0]  # current, max

    def pub(states):
        inflight[0] += 1
        inflight[1] = max(inflight[1], inflight[0])
        out.append(states)
        inflight[0] -= 1

    c = StatusComposer(pub, "MODE_AUTO")
    cur = ["IDLE"]
    c.add_contributor(lambda: list(cur))

    def work(i):
        for k in range(50):
            cur[:] = [f"V{i}_{k}"]
            c.refresh()

    ts = [threading.Thread(target=work, args=(i,)) for i in range(4)]
    [t.start() for t in ts]
    [t.join() for t in ts]
    assert c.states() == out[-1]
    assert all(a != b for a, b in zip(out, out[1:]))
    assert inflight[1] == 1

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


def test_contributor_mode_ignored(caplog):
    c, out = make()
    c.add_contributor(lambda: ["MODE_MANUAL", "IDLE"])
    with caplog.at_level(logging.WARNING):
        c.refresh()
        c.refresh()
    assert out[-1] == ["MODE_AUTO", "IDLE"]
    assert len([r for r in caplog.records if "MODE_MANUAL" in r.getMessage()]) == 1


def test_never_emits_ready():
    c, out = make()
    c.refresh()
    assert out[-1] == ["MODE_AUTO"]
    assert "READY" not in out[-1] and "NOT_READY" not in out[-1]


def test_contributor_exception_skipped_and_logged(caplog):
    c, out = make()

    def bad():
        raise RuntimeError("boom")

    c.add_contributor(bad)
    c.add_contributor(lambda: ["IDLE"])
    with caplog.at_level(logging.ERROR):
        c.refresh()
    assert out[-1] == ["MODE_AUTO", "IDLE"]
    assert "boom" in caplog.text


def test_invalid_mode():
    with pytest.raises(ValueError):
        StatusComposer(lambda s: None, "MODE_BOGUS")
    c, _ = make()
    with pytest.raises(ValueError):
        c.set_mode("READY")


def test_concurrent_refresh_publishes_in_order():
    c, out = make()
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

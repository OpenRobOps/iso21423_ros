import threading
import time

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from iso21423 import ActionContext
from iso21423_ros import oro
from iso21423_ros.oro import CustomCommandHandler, CustomDataSource, parse_key_value


class FakePub:
    def __init__(self):
        self.calls, self.fail = [], False

    def publish_extension(self, resource, payload):
        if self.fail:
            raise OSError("broker down")
        self.calls.append((resource, payload))


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    n = Node("test_oro")
    yield n
    n.destroy_node()


def msg(s):
    m = String()
    m.data = s
    return m


def wait_for(cond, t=2.0):
    end = time.monotonic() + t
    while not cond() and time.monotonic() < end:
        time.sleep(0.01)
    return cond()


def test_parse_key_value():
    assert parse_key_value("a=b") == ("a", "b")
    assert parse_key_value("a=b=c") == ("a", "b=c")
    assert parse_key_value("a=") == ("a", "")
    assert parse_key_value("=b") is None
    assert parse_key_value("nokv") is None


def test_custom_data_batched(node):
    pub = FakePub()
    src = CustomDataSource(node, pub, {"batch_ms": 50})
    for s in ("a=1", "b=2", "a=3", "junk"):
        src._on_msg(msg(s))
    assert wait_for(lambda: pub.calls)
    (res, payload), = pub.calls
    assert res == "customData"
    assert payload["values"] == {"a": "3", "b": "2"}
    assert isinstance(payload["timestamp"], str)
    src._on_msg(msg("c=4"))  # a new window starts after a flush
    assert wait_for(lambda: len(pub.calls) == 2)
    assert pub.calls[1][1]["values"] == {"c": "4"}


def test_custom_data_close_flushes_last_batch(node):
    pub = FakePub()
    src = CustomDataSource(node, pub, {"batch_ms": 60000})
    src._on_msg(msg("a=1"))
    assert not pub.calls
    src.close()
    assert pub.calls[0][1]["values"] == {"a": "1"}
    src.close()  # idempotent: nothing pending
    assert len(pub.calls) == 1


def test_custom_data_publish_failure_is_logged(node, logcap):
    pub = FakePub()
    pub.fail = True
    src = CustomDataSource(node, pub, {"batch_ms": 20})
    src._on_msg(msg("a=1"))
    assert wait_for(lambda: "customData publish failed" in logcap.text)
    pub.fail = False
    src._on_msg(msg("b=2"))  # still usable afterwards
    assert wait_for(lambda: pub.calls)


def test_custom_data_registration_idempotent(node):
    CustomDataSource(node, FakePub(), {})
    CustomDataSource(node, FakePub(), {})  # same config again: no raise


def ctx():
    return ActionContext(None, None, None, threading.Event(), lambda *a, **k: None)


def test_custom_command_publishes_and_validates(node):
    got = []
    node.create_subscription(String, "/inorbit/custom_command", lambda m: got.append(m.data), 10)
    h = CustomCommandHandler(node, {})
    r = h.execute({"properties": {"command": 5}}, ctx())
    assert (r.outcome, r.reason, r.message) == ("aborted", "MALFORMED_REQUEST",
                                               "properties.command must be a string")
    assert h.execute({"properties": {}}, ctx()).reason == "MALFORMED_REQUEST"
    assert h.execute({"properties": "x"}, ctx()).reason == "MALFORMED_REQUEST"
    r = h.execute({"properties": {"command": "go"}}, ctx())
    assert r.outcome == "succeeded"
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    assert wait_for(lambda: (ex.spin_once(timeout_sec=0.05), got)[1])
    assert got == ["go"]

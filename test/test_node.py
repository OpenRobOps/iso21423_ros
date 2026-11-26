"""Connector node end to end: in-process rclpy node against a Mosquitto started by the test."""
import contextlib
import json
import shutil
import socket
import subprocess
import threading
import time
import uuid
from pathlib import Path

import paho.mqtt.client as mqtt
import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from conftest import Spinner
from test_nav import FakeNav
from iso21423_ros.handlers.dock import UndockHandler
from iso21423_ros.node import ConnectorNode

PKG = Path(__file__).resolve().parent.parent
ROOT = "/ISO_21423/v1/IMR"
CCS = "2385eed2-86ca-4dc9-8f17-dac062ce9a08"
WAIT = 30.0


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(autouse=True)
def state_dir(tmp_path, monkeypatch):
    monkeypatch.setenv("ISO21423_STATE_DIR", str(tmp_path / "state"))


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Mosquitto:
    def __init__(self, port, tmp):
        conf = tmp / f"mosquitto-{port}.conf"
        conf.write_text(f"listener {port} 127.0.0.1\nallow_anonymous true\n")
        exe = shutil.which("mosquitto") or "/usr/sbin/mosquitto"
        self.p = subprocess.Popen([exe, "-c", str(conf)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        deadline = time.monotonic() + 10
        while True:
            try:
                socket.create_connection(("127.0.0.1", port), 0.5).close()
                return
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.05)

    def stop(self):
        self.p.kill()
        self.p.wait()


@pytest.fixture
def broker(tmp_path):
    port = free_port()
    m = Mosquitto(port, tmp_path)
    yield f"mqtt://127.0.0.1:{port}"
    m.stop()


class Fleet:
    """Raw paho client standing in for the fleet manager; (re)subscribes on every connect."""

    def __init__(self, url, *filters):
        host, port = url.removeprefix("mqtt://").split(":")
        kw = {"client_id": f"fleet-{uuid.uuid4().hex[:8]}"}
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, **kw) if hasattr(mqtt, "CallbackAPIVersion") \
            else mqtt.Client(**kw)
        self.msgs, self.cond, self.filters = [], threading.Condition(), filters
        self.c.on_connect = lambda c, *_: [c.subscribe(f, 2) for f in self.filters]
        self.c.on_message = self._on_message
        self.c.reconnect_delay_set(1, 1)
        self.c.connect_async(host, int(port))
        self.c.loop_start()

    def _on_message(self, c, ud, m):
        with self.cond:
            self.msgs.append((m.topic, json.loads(m.payload) if m.payload else None))
            self.cond.notify_all()

    def wait(self, pred, what):
        with self.cond:
            assert self.cond.wait_for(lambda: pred(), WAIT), f"timed out waiting for {what}; got {self.msgs[-10:]}"

    def on(self, suffix):
        with self.cond:
            return [p for t, p in self.msgs if t.endswith(suffix)]

    def stop(self):
        self.c.disconnect()
        self.c.loop_stop()


def base_params(url, uid, **extra):
    p = {"entity_uuid": uid, "manufacturer_name": "Acme", "broker.url": url, "ccs.id": CCS,
         "identity.model": "m1", "identity.serial_number": "sn1", "identity.height": 0.4,
         "identity.footprint": [0.3, 0.2, 0.3, -0.2, -0.3, -0.2]}
    p.update(extra)
    return p


@contextlib.contextmanager
def connector(params):
    node = ConnectorNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
    spinner = Spinner(node)
    starter = threading.Thread(target=node.start, daemon=True)
    starter.start()
    try:
        yield node
    finally:
        node.shutdown()
        starter.join(15)  # a connect attempt in flight ends within its 10 s timeout
        spinner.stop()
        node.destroy_node()
        assert not starter.is_alive(), "node.start() still running after shutdown"


def test_capabilities_derived_from_config(broker, monkeypatch):
    monkeypatch.setattr(UndockHandler, "available", staticmethod(lambda: False))
    uid = str(uuid.uuid4())
    fleet = Fleet(broker, f"{ROOT}/{uid}/identity", f"{ROOT}/{uid}/status")
    params = base_params(broker, uid, **{
        "sources.odometry.enabled": True, "sources.batteryStatus.topic": "/battery_state",
        "sources.globalPlan.enabled": False,
        "handlers.move.enabled": True, "handlers.undock.enabled": True, "handlers.pauseImr.enabled": True,
        "handlers.dock.enabled": False, "extensions.customData.enabled": True})
    try:
        with connector(params):
            fleet.wait(lambda: any(fleet.on("/identity")), "identity")
            fleet.wait(lambda: any(fleet.on("/status")), "status")
            ident = fleet.on("/identity")[-1]
            assert ident["capabilities"]["provides"] == ["status", "odometry", "batteryStatus", "customData"]
            assert ident["capabilities"]["accepts"]["requests"] == ["move", "pauseImr"]
            d = ident["details"]
            assert (d["imrModel"], d["imrSerialNumber"], d["imrHeight"]) == ("m1", "sn1", 0.4)
            assert d["imrFootprint"] == d["imrWorkingArea"] == [
                {"x": 0.3, "y": 0.2}, {"x": 0.3, "y": -0.2}, {"x": -0.3, "y": -0.2}]
            assert d["softwareVersions"][0]["moduleName"] == "iso21423_ros"
            assert fleet.on("/status")[-1]["states"][0] == "MODE_AUTO"
    finally:
        fleet.stop()


def test_default_config_file_enables_nav2_profile():
    uid = str(uuid.uuid4())
    node = ConnectorNode(cli_args=["--ros-args", "--params-file", str(PKG / "config" / "nav2_default.yaml"),
                                   "-p", f"entity_uuid:={uid}"])
    try:
        assert set(node.config.sources) == {"odometry", "batteryStatus", "globalPlan", "localTrajectory"}
        assert set(node.config.handlers) == {"move", "dock", "undock", "pauseImr", "resumeImr"}
        assert node.config.execution_policy == "preempt"
    finally:
        node.destroy_node()


def test_bad_hook_fails_fast():
    with pytest.raises(ValueError, match=r"^handlers.move.class: cannot load 'nope:X'"):
        ConnectorNode(parameter_overrides=[Parameter("entity_uuid", value=str(uuid.uuid4())),
                                           Parameter("handlers.move.class", value="nope:X")])
    with pytest.raises(ValueError, match=r"^sources.thermo.class: required"):
        ConnectorNode(parameter_overrides=[Parameter("entity_uuid", value=str(uuid.uuid4())),
                                           Parameter("sources.thermo.topic", value="/t")])


def test_vendor_custom_action_example(broker):
    example = PKG / "config" / "example_vendor_hook.py"
    assert len(example.read_text().splitlines()) < 50
    uid = str(uuid.uuid4())
    fleet = Fleet(broker, f"{ROOT}/{uid}/identity", f"{ROOT}/{uid}/request/+/status")
    params = base_params(broker, uid, **{"handlers.beep.class": f"{example}:BeepHandler"})
    try:
        with connector(params):
            fleet.wait(lambda: any(fleet.on("/identity")), "identity")
            assert fleet.on("/identity")[-1]["capabilities"]["accepts"]["requests"] == ["beep"]
            req = {"destination": uid, "source": "11111111-1111-4111-8111-111111111111", "sequenceId": 1,
                   "timestamp": "2026-11-25T12:00:00.000Z",
                   "details": [{"type": "beep", "version": "1.0", "properties": {"duration_s": 0.1}}]}
            fleet.c.publish(f"{ROOT}/{uid}/request/{uuid.uuid4()}", json.dumps(req), 2, True)
            fleet.wait(lambda: any(s and s["status"] == "SUCCEEDED" for s in fleet.on("/status")), "SUCCEEDED")
    finally:
        fleet.stop()


def test_starts_when_broker_comes_up_later(tmp_path):
    port = free_port()
    url = f"mqtt://127.0.0.1:{port}"
    uid = str(uuid.uuid4())
    with connector(base_params(url, uid)):
        time.sleep(3)
        m = Mosquitto(port, tmp_path)
        fleet = Fleet(url, f"{ROOT}/{uid}/identity")
        try:
            fleet.wait(lambda: any(fleet.on("/identity")), "identity")
        finally:
            fleet.stop()
            m.stop()


def test_unregister_on_shutdown(broker):
    uid = str(uuid.uuid4())
    fleet = Fleet(broker, f"{ROOT}/{uid}/identity", f"{ROOT}/{uid}/status")
    try:
        with connector(base_params(broker, uid, **{"handlers.move.enabled": True})):
            fleet.wait(lambda: any(fleet.on("/identity")), "identity")
        fleet.wait(lambda: [s["states"] for s in fleet.on("/status") if s][-1:] == [["OFFLINE"]], "OFFLINE")
        fleet.wait(lambda: fleet.on("/identity")[-1] is None, "identity cleared")
    finally:
        fleet.stop()
    late = Fleet(broker, f"{ROOT}/{uid}/#")  # what a fresh subscriber is handed: only the tombstone
    time.sleep(1.0)
    late.stop()
    assert [(t.rsplit("/", 1)[1], p["states"]) for t, p in late.msgs] == [("status", ["OFFLINE"])]


def test_shutdown_while_broker_down_exits_cleanly():
    url = f"mqtt://127.0.0.1:{free_port()}"
    t0 = time.monotonic()
    with connector(base_params(url, str(uuid.uuid4()))) as node:
        time.sleep(0.5)
    assert time.monotonic() - t0 < 15 and node.entity is None  # the helper also joined node.start


def test_shutdown_cancels_in_flight_move(broker):
    nav_node = Node("fake_nav_for_connector")
    fake = FakeNav(nav_node, "/connector_test_nav")
    fake.duration = 60.0  # still driving when we shut down
    spinner = Spinner(nav_node)
    uid = str(uuid.uuid4())
    fleet = Fleet(broker, f"{ROOT}/{uid}/identity", f"{ROOT}/{uid}/request/+/status")
    params = base_params(broker, uid, **{"handlers.move.action": "/connector_test_nav"})
    try:
        with connector(params):
            fleet.wait(lambda: any(fleet.on("/identity")), "identity")
            req = {"destination": uid, "source": "11111111-1111-4111-8111-111111111111", "sequenceId": 1,
                   "timestamp": "2026-11-25T12:00:00.000Z",
                   "details": [{"type": "move", "version": "1.0",
                                "properties": {"location": {"ccsId": CCS, "x": 5.0, "y": 0.0, "z": 0}}}]}
            fleet.c.publish(f"{ROOT}/{uid}/request/{uuid.uuid4()}", json.dumps(req), 2, True)
            fleet.wait(lambda: len(fake.goals) == 1, "goal at the fake Nav2 server")
            t0 = time.monotonic()
        took = time.monotonic() - t0  # shutdown, inside the helper's exit
        assert fake.cancels == fake.goals, "the in-flight goal was not cancelled"
        final = [st for st in fleet.on("/status") if st][-1]
        assert final["status"] == "ABORTED" and "connector shutting down" in json.dumps(final), final
        assert took < 5, f"shutdown took {took:.1f}s"
    finally:
        fleet.stop()
        spinner.stop()
        nav_node.destroy_node()


def test_footprint_from_nav2(broker):
    costmap = Node("global_costmap", namespace="global_costmap",
                   parameter_overrides=[Parameter("footprint", value="[[0.5, 0.25], [0.5, -0.25], [-0.5, 0.0]]")],
                   automatically_declare_parameters_from_overrides=True)
    spinner = Spinner(costmap)
    uid = str(uuid.uuid4())
    fleet = Fleet(broker, f"{ROOT}/{uid}/identity")
    try:
        with connector(base_params(broker, uid, **{"identity.footprint_from_nav2": True})):
            fleet.wait(lambda: any(fleet.on("/identity")), "identity")
            assert fleet.on("/identity")[-1]["details"]["imrFootprint"] == [
                {"x": 0.5, "y": 0.25}, {"x": 0.5, "y": -0.25}, {"x": -0.5, "y": 0.0}]
    finally:
        fleet.stop()
        spinner.stop()
        costmap.destroy_node()

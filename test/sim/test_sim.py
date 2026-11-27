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

"""Sim integration: the connector against flatland + Nav2, driven over raw MQTT/JSON as a fleet manager.

Run by scripts/sim_test.sh (host pytest, not colcon). Tests run in file order against one sim and
start from wherever the previous one left the robot. Targets are free cells of flatland's
sample_map (20x20 m office, robot spawns at (5, 15) in office A), each >= 0.8 m from any wall.
"""
import itertools
import json
import math
import threading
import time
import uuid

import paho.mqtt.client as mqtt
import pytest

BROKER = ("localhost", 18830)
UID = "5f0c2a1e-9b7d-4c3a-8e21-0d4f6b7a9c11"  # test/sim/flatland_test.yaml
CCS = "7d3e9f20-4b1a-4c6e-9a5d-2e8f1b0c3a44"
ROOT = f"/ISO_21423/v1/IMR/{UID}"
FLEET = str(uuid.uuid4())
STARTUP = 240.0  # sim + Nav2 + connector up from cold
WAIT = 60.0
NAV = 180.0  # a cross-map drive (~75 s locally; CI runners are slower)

OFFICE_A = (3.0, 13.0)
HALL_EAST = (15.0, 10.0)
HALL_WEST = (3.0, 10.0)
OFFICE_B = (16.0, 15.0)
HALL_MID = (10.0, 10.0)

_seq = itertools.count(1)


class Fleet:
    def __init__(self):
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"fleet-{uuid.uuid4().hex[:8]}")
        self.msgs = []
        self.cond = threading.Condition()
        self.c.on_connect = lambda c, *a: c.subscribe(f"{ROOT}/#", 2)
        self.c.on_message = self._on_message
        self.c.reconnect_delay_set(1, 2)
        deadline = time.monotonic() + STARTUP
        while True:  # the broker container may still be starting
            try:
                self.c.connect(*BROKER)
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(1)
        self.c.loop_start()

    def _on_message(self, c, ud, m):
        with self.cond:
            self.msgs.append((m.topic, json.loads(m.payload) if m.payload else None, m.retain))
            self.cond.notify_all()

    def on(self, suffix):
        with self.cond:
            return [p for t, p, _ in self.msgs if t == ROOT + suffix and p is not None]

    def wait(self, pred, what, timeout=WAIT):
        with self.cond:
            if not self.cond.wait_for(pred, timeout):
                raise AssertionError(f"timed out after {timeout:.0f}s waiting for {what}; "
                                     f"states={self.states()} last={self.msgs[-5:]}")

    # --- views ---
    def statuses(self, ruuid):
        return [p["status"] for p in self.on(f"/request/{ruuid}/status")]

    def states(self):
        s = self.on("/status")
        return s[-1]["states"] if s else []

    def xy(self):
        o = self.on("/odometry")
        p = o[-1]["pose"]["locationPoint"] if o else None
        return (p["x"], p["y"]) if p else None

    # --- requests ---
    def send(self, type_, props=None):
        seq = next(_seq)
        req = {"destination": UID, "source": FLEET, "sequenceId": seq,
               "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime()),
               "details": [{"type": type_, "version": "1.0", "properties": props or {}}]}
        ruuid = str(uuid.uuid4())
        self.c.publish(f"{ROOT}/request/{ruuid}", json.dumps(req), 2, True)
        return ruuid, seq

    def move(self, xy):
        return self.send("move", {"location": loc(xy)})

    def wait_status(self, ruuid, status, timeout=WAIT):
        self.wait(lambda: status in self.statuses(ruuid), f"{status} for {ruuid} (got {self.statuses(ruuid)})",
                  timeout)

    def wait_terminal(self, ruuid, timeout):
        terminal = {"SUCCEEDED", "CANCELED", "ABORTED"}
        self.wait(lambda: terminal & set(self.statuses(ruuid)), f"terminal status for {ruuid}", timeout)
        return self.statuses(ruuid)[-1]

    def wait_moving(self, ruuid, start):
        """Until the robot has left `start` by 0.3 m: the goal is mid-way, not about to be accepted."""
        def ended():
            return {"SUCCEEDED", "CANCELED", "ABORTED"} & set(self.statuses(ruuid))
        self.wait(lambda: ended() or dist(self.xy(), start) > 0.3, "the robot to move", WAIT)
        assert not ended(), f"{ruuid} ended before moving: {self.on(f'/request/{ruuid}/status')[-1]}"


def loc(xy):
    return {"ccsId": CCS, "x": xy[0], "y": xy[1], "z": 0.0}


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def arrive(fleet, ruuid, target):
    assert fleet.wait_terminal(ruuid, NAV) == "SUCCEEDED", fleet.statuses(ruuid)
    # odometry is published at 2 Hz: give the last sample a moment to land
    fleet.wait(lambda: dist(fleet.xy(), target) < 0.5, f"odometry near {target} (at {fleet.xy()})", 5)


@pytest.fixture(scope="module")
def fleet():
    f = Fleet()
    try:
        f.wait(lambda: f.on("/identity"), "identity", STARTUP)
        f.wait(lambda: f.on("/odometry"), "first odometry (localized)", STARTUP)
        # Readiness gate. Odometry comes up about a second before bt_navigator is active, and an
        # inactive Nav2 rejects goals: the move ends ABORTED "goal rejected", which is correct
        # connector behaviour. The gate hides that path from the tests below (no test covers it):
        # probe with a move in place until Nav2 takes one.
        deadline = time.monotonic() + STARTUP
        while f.wait_terminal(f.move(f.xy())[0], WAIT) != "SUCCEEDED":
            assert time.monotonic() < deadline, "Nav2 never became active"
            time.sleep(1)
        yield f
    finally:
        f.c.disconnect()
        f.c.loop_stop()


def test_identity_and_telemetry(fleet):
    late = Fleet()  # a subscriber arriving after registration is handed the retained identity
    try:
        late.wait(lambda: late.on("/identity"), "identity for a late subscriber")
        assert [r for t, _, r in late.msgs if t == ROOT + "/identity"][0], "identity not retained"
    finally:
        late.c.disconnect()
        late.c.loop_stop()
    ident = fleet.on("/identity")[-1]
    assert ident["id"] == UID
    caps = ident["capabilities"]
    assert {"status", "odometry", "batteryStatus"} <= set(caps["provides"])
    assert {"move", "dock", "pauseImr", "resumeImr"} <= set(caps["accepts"]["requests"])
    odo = fleet.on("/odometry")[-1]
    assert odo["pose"]["locationPoint"]["ccsId"] == CCS
    fleet.wait(lambda: fleet.on("/batteryStatus"), "batteryStatus")
    assert 0 <= fleet.on("/batteryStatus")[-1]["batterySoc"] <= 1
    assert fleet.states()[:1] == ["MODE_AUTO"]


def test_move_round_trip(fleet):
    ruuid, _ = fleet.move(OFFICE_A)
    arrive(fleet, ruuid, OFFICE_A)
    assert fleet.statuses(ruuid)[:3] == ["RECEIVED", "ACCEPTED", "EXECUTING"]


def test_cancel_move(fleet):
    start = fleet.xy()
    ruuid, seq = fleet.move(HALL_EAST)
    fleet.wait_moving(ruuid, start)
    cancel, _ = fleet.send("cancelRequest", {"source": FLEET, "requestId": seq})
    assert fleet.wait_terminal(cancel, WAIT) == "SUCCEEDED"
    assert fleet.wait_terminal(ruuid, WAIT) == "CANCELED"
    at = fleet.xy()
    time.sleep(3)
    assert dist(fleet.xy(), at) < 0.3, "the robot kept driving after cancel"
    assert dist(at, HALL_EAST) > 1.0


def test_pause_resume_move(fleet):
    start = fleet.xy()
    ruuid, _ = fleet.move(OFFICE_B)
    fleet.wait_moving(ruuid, start)
    pause, _ = fleet.send("pauseImr")
    assert fleet.wait_terminal(pause, WAIT) == "SUCCEEDED"
    fleet.wait(lambda: "PAUSED" in fleet.states(), "PAUSED state")
    at = fleet.xy()
    time.sleep(3)
    assert dist(fleet.xy(), at) < 0.3, "the robot kept driving while paused"
    assert "PAUSED" in fleet.states()
    assert fleet.statuses(ruuid)[-1] == "EXECUTING"
    resume, _ = fleet.send("resumeImr")
    assert fleet.wait_terminal(resume, WAIT) == "SUCCEEDED"
    fleet.wait(lambda: "PAUSED" not in fleet.states(), "PAUSED cleared")
    arrive(fleet, ruuid, OFFICE_B)


def test_dock_falls_back_to_navigate(fleet):
    # dockId asks for the docking server; flatland runs none, so auto mode navigates to dockLocation
    ruuid, _ = fleet.send("dock", {"dockId": "A", "dockLocation": loc(HALL_MID)})
    fleet.wait(lambda: "DOCKING" in fleet.states(), "DOCKING state")
    arrive(fleet, ruuid, HALL_MID)
    fleet.wait(lambda: "DOCKING" not in fleet.states(), "DOCKING cleared")


def test_preempt_new_move(fleet):
    start = fleet.xy()
    first, _ = fleet.move(HALL_EAST)
    fleet.wait_moving(first, start)
    second, _ = fleet.move(HALL_WEST)
    assert fleet.wait_terminal(first, WAIT) == "CANCELED"
    arrive(fleet, second, HALL_WEST)

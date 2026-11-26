#!/usr/bin/env bash
# Simulation integration test: flatland + Nav2, Mosquitto and the connector (test/sim/docker-compose.yml),
# driven by test/sim/test_sim.py on the host acting as the fleet manager. Always tears down.
# Extra args are passed to pytest. PYTHONPATH is dropped so a sourced host ROS does not leak in.
set -euo pipefail
cd "$(dirname "$0")/.."
export CONNECTOR_IMAGE=iso21423-ros:sim
docker build --build-context iso21423=../iso21423 --target runtime -t "$CONNECTOR_IMAGE" -f docker/Dockerfile .

VENV=test/sim/.venv
[ -x $VENV/bin/pytest ] || { python3 -m venv $VENV && $VENV/bin/pip install -q pytest paho-mqtt; }

COMPOSE="docker compose -f test/sim/docker-compose.yml -p iso21423-ros-sim"
down() {
  [ "$rc" = 0 ] || $COMPOSE logs --no-color --tail 200 connector
  $COMPOSE down -v --remove-orphans
}
rc=1
trap down EXIT
$COMPOSE up -d
env -u PYTHONPATH $VENV/bin/pytest -v -p no:cacheprovider --confcutdir test/sim test/sim "$@" && rc=0
exit $rc

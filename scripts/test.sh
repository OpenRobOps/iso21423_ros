#!/usr/bin/env bash
# Run the test suite in the dev image. ROS_DISTRO=humble scripts/test.sh for Humble.
# Extra args are passed to pytest.
set -euo pipefail
cd "$(dirname "$0")/.."
ROS_DISTRO=${ROS_DISTRO:-jazzy}
docker build --build-context iso21423=../iso21423 --build-arg ROS_DISTRO=$ROS_DISTRO \
  --target dev -t iso21423-ros:dev-$ROS_DISTRO -f docker/Dockerfile .
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD":/ws/src/iso21423_ros iso21423-ros:dev-$ROS_DISTRO bash -c \
  'source /opt/ros/$0/setup.bash && cd /ws && colcon build && colcon test --event-handlers console_direct+ ${1:+--pytest-args "$@"} && colcon test-result --verbose' \
  "$ROS_DISTRO" "$@"

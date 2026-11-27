#!/usr/bin/env bash
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

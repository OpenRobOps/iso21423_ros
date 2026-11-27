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

"""move: ISO location (CCS) -> NavigateToPose via the shared node.nav runner."""
import math

from iso21423_ros.mappings import nav_goal_pose


class MoveHandler:
    def __init__(self, node, params):
        self.node = node
        action = params.get("action", "/navigate_to_pose")  # the node builds node.nav with it
        if not isinstance(action, str) or not action:
            raise ValueError(f"handlers.move.action: expected an action name, got {action!r}")

    def execute(self, action, ctx):
        props = action["properties"]
        if "location" not in props:
            return ctx.aborted("GENERAL_FAILURE", "move: properties.location is required")
        tol = props.get("toleranceRadius")
        number = isinstance(tol, (int, float)) and not isinstance(tol, bool)
        if tol is not None and not (number and math.isfinite(tol) and tol >= 0):
            return ctx.aborted("GENERAL_FAILURE", f"move: toleranceRadius must be a non-negative number, got {tol!r}")
        yaw = (props.get("orientation") or {}).get("yaw")
        pose = nav_goal_pose(self.node.ccs, props["location"], yaw, self.node.map_frame)
        return self.node.nav.navigate(pose, ctx, tolerance_radius=tol)

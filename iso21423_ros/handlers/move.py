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

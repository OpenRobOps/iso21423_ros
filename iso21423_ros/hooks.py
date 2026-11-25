"""Load pluggable Source/Handler classes from "package.module:ClassName" specs."""
import importlib

DEFAULT_SOURCES = {
    "odometry": "iso21423_ros.sources.odometry:OdometrySource",
    "batteryStatus": "iso21423_ros.sources.battery:BatterySource",
    "globalPlan": "iso21423_ros.sources.paths:GlobalPlanSource",
    "localTrajectory": "iso21423_ros.sources.paths:LocalTrajectorySource",
}
DEFAULT_HANDLERS = {
    "move": "iso21423_ros.handlers.move:MoveHandler",
    "dock": "iso21423_ros.handlers.dock:DockHandler",
    "undock": "iso21423_ros.handlers.dock:UndockHandler",
    "pauseImr": "iso21423_ros.handlers.pause:PauseHandler",
    "resumeImr": "iso21423_ros.handlers.pause:ResumeHandler",
}


def load_class(spec: str, key: str) -> type:
    try:
        module, _, name = spec.partition(":")
        if not module or not name:
            raise ValueError("expected 'package.module:ClassName'")
        return getattr(importlib.import_module(module), name)
    except (ImportError, AttributeError, ValueError) as e:
        raise ValueError(f"{key}: cannot load '{spec}': {e}") from e

"""iso21423_connector: the ROS 2 node that makes this robot an ISO 21423 IMR."""
import array
import ast
import importlib.metadata
import logging
import signal
import sys
import threading
import time

import rclpy
import tf2_ros
from iso21423 import Client, Iso21423Error, policies
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from iso21423_ros import config, credentials
from iso21423_ros.ccs import CcsFrame
from iso21423_ros.hooks import DEFAULT_HANDLERS, DEFAULT_SOURCES, load_class
from iso21423_ros.nav import NavRunner
from iso21423_ros.status import StatusComposer

log = logging.getLogger(__name__)

DEFAULT_EXTENSIONS = {"customData": "iso21423_ros.oro:CustomDataSource"}
RETRY_S = 10
NAV2_FOOTPRINT_SERVICE = "/global_costmap/global_costmap/get_parameters"


def _enabled(section, entries, defaults):
    """{name: (cls, params)} for every entry that has params and not `enabled: false`."""
    out = {}
    for name, params in entries.items():
        if params.get("enabled", True) is False:
            continue
        spec = params.get("class") or defaults.get(name)
        if not spec:
            raise ValueError(f"{section}.{name}.class: required (no default {section[:-1]} named {name!r})")
        cls = load_class(spec, f"{section}.{name}.class")
        out[name] = (cls, {k: v for k, v in params.items() if k not in ("enabled", "class")})
    return out


def _guarded(steps):
    """Run every (what, fn) step; one failure doesn't skip the rest."""
    for what, fn in steps:
        try:
            fn()
        except Exception:
            log.exception("shutdown: %s failed", what)


def _version():
    try:
        return importlib.metadata.version("iso21423_ros")
    except importlib.metadata.PackageNotFoundError:
        return "unknown"


class ConnectorNode(Node):
    def __init__(self, **kwargs):
        super().__init__("iso21423_connector", automatically_declare_parameters_from_overrides=True, **kwargs)
        try:
            self._configure()
        except Exception:
            Node.destroy_node(self)  # don't leak a half-built node (bad config fails fast)
            raise

    def _configure(self):
        flat = {k: list(p.value) if isinstance(p.value, array.array) else p.value
                for k, p in self.get_parameters_by_prefix("").items()}
        self.config = cfg = config.from_params(flat)
        self._sources = _enabled("sources", cfg.sources, DEFAULT_SOURCES)
        self._extensions = _enabled("extensions", cfg.extensions, DEFAULT_EXTENSIONS)
        self._handlers = {n: h for n, h in _enabled("handlers", cfg.handlers, DEFAULT_HANDLERS).items()
                          if getattr(h[0], "available", lambda: True)()}  # e.g. undock on Humble
        self.ccs = CcsFrame(cfg.ccs.id, cfg.ccs.reference_points)
        self.map_frame, self.base_frame = cfg.ccs.map_frame, cfg.base_frame
        self.tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.status = self.nav = self.entity = None
        self.client = None
        self.hooks = []
        self._stop = threading.Event()
        self._lock = threading.Lock()  # serialises the post-connect build against shutdown()
        self._closed = False

    # ---- startup -------------------------------------------------------------------------------
    def start(self):
        """Blocking: credentials, broker connect (retried), registration, hooks. Returns early on shutdown.
        Needs an executor spinning this node (footprint_from_nav2 calls a service)."""
        try:
            self._start()
        except Iso21423Error:
            if not self._stop.is_set():
                raise
            log.info("startup interrupted by shutdown")

    def _start(self):
        cfg = self.config
        if self._stop.is_set():
            return
        creds = credentials.resolve(cfg, stop=self._stop)
        details = self._details()
        provides = ["status", *self._sources, *self._extensions]
        accepts = list(self._handlers)
        while True:
            if self._stop.is_set():
                return
            try:
                client = Client.connect(creds.url, username=creds.username or None,
                                        password=creds.password or None, tls=creds.tls)
                entity = client.register_self_entity(
                    cfg.entity_uuid, "IMR", manufacturer_name=cfg.manufacturer_name, details=details,
                    provides=provides, accepts=accepts,
                    execution_policy=getattr(policies, cfg.execution_policy)())
                break
            except Exception as e:  # broker down, refused, DNS...: retry, never crash
                log.warning("broker %s: %s; retrying in %ss", creds.url, e, RETRY_S)
                self._stop.wait(RETRY_S)
        with self._lock:
            if self._closed:  # shutdown() ran while we were connecting: it never saw this client
                _guarded([("unregister", entity.unregister), ("client close", client.close)])
                return
            self.client, self.entity = client, entity
            log.info("registered %s on %s: provides %s, accepts %s", cfg.entity_uuid, creds.url, provides, accepts)
            self._build(entity)

    def _build(self, entity):
        cfg = self.config
        self.status = StatusComposer(self._publish_status, cfg.mode)
        move = cfg.handlers.get("move", {})
        self.nav = NavRunner(self, move.get("action", "/navigate_to_pose"))
        for cls, params in [*self._sources.values(), *self._extensions.values()]:
            self.hooks.append(cls(self, entity, params))
        handlers = {}
        for name, (cls, params) in self._handlers.items():
            handlers[name] = h = cls(self, params)
            self.hooks.append(h)
        for h in self.hooks:
            if callable(getattr(h, "states", None)):
                self.status.add_contributor(h.states)
        self.status.add_contributor(self.nav.states)
        for name, h in handlers.items():
            entity.on_request(name, h.execute)
        entity.start_serving()
        self.status.refresh()

    def _publish_status(self, states):
        if not self._closed:  # after unregister the OFFLINE tombstone must stay last
            self.entity.publish_status(states)

    def _details(self):
        ident = self.config.identity
        fp = (self._nav2_footprint() if ident.footprint_from_nav2 else None) or ident.footprint
        fp = [{"x": float(x), "y": float(y)} for x, y in fp]
        return {"imrModel": ident.model, "imrSerialNumber": ident.serial_number, "imrFootprint": fp,
                "imrWorkingArea": fp, "imrHeight": float(ident.height),
                "softwareVersions": [{"moduleName": "iso21423_ros", "moduleVersion": _version()}]}

    def _nav2_footprint(self, timeout=5.0):
        """Nav2's global costmap footprint (a string "[[x, y], ...]"), or None (logged)."""
        deadline = time.monotonic() + timeout
        cli = self.create_client(GetParameters, NAV2_FOOTPRINT_SERVICE, callback_group=ReentrantCallbackGroup())
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                raise TimeoutError(f"{NAV2_FOOTPRINT_SERVICE} not available")
            done = threading.Event()
            fut = cli.call_async(GetParameters.Request(names=["footprint"]))
            fut.add_done_callback(lambda _: done.set())
            if not done.wait(max(0.0, deadline - time.monotonic())):
                raise TimeoutError(f"{NAV2_FOOTPRINT_SERVICE} did not answer")
            pts = [(float(x), float(y)) for x, y in ast.literal_eval(fut.result().values[0].string_value)]
            if len(pts) < 3:
                raise ValueError(f"footprint has {len(pts)} points")
            return pts
        except Exception as e:
            log.warning("footprint_from_nav2: %s; using identity.footprint", e)
            return None
        finally:
            self.destroy_client(cli)

    # ---- shutdown ------------------------------------------------------------------------------
    def shutdown(self):
        """Idempotent: stop startup, flush sources, unregister (OFFLINE tombstone), close the client."""
        self._stop.set()
        with self._lock:  # waits for a build in progress; a connect in progress cleans up after itself
            if self._closed:
                return
            self._closed = True
        steps = [("destroy subscriptions/timers", self._destroy_entities)]
        steps += [(f"close {type(h).__name__}", h.close) for h in self.hooks if callable(getattr(h, "close", None))]
        if self.entity:
            steps.append(("unregister", self.entity.unregister))
        if self.client:
            steps.append(("client close", self.client.close))
        _guarded(steps)

    def _destroy_entities(self):
        for s in list(self.subscriptions):
            self.destroy_subscription(s)
        for t in list(self.timers):
            self.destroy_timer(t)

    def destroy_node(self):
        self.shutdown()
        return super().destroy_node()


def main(args=None):
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] [%(name)s] %(message)s")
    # our own SIGINT/SIGTERM handling: rclpy's raises KeyboardInterrupt at an arbitrary point,
    # which could cut the shutdown sequence (and the OFFLINE tombstone) short
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    done, failed = threading.Event(), []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: done.set())
    try:
        node = ConnectorNode()
    except ValueError as e:
        log.error("configuration error: %s", e)
        rclpy.try_shutdown()
        sys.exit(1)

    def startup():
        try:
            node.start()
        except Exception:
            log.exception("startup failed")
            failed.append(True)
            done.set()

    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=startup, daemon=True).start()
    try:
        while not done.is_set() and rclpy.ok():
            ex.spin_once(timeout_sec=0.1)
    finally:
        node.destroy_node()
        ex.shutdown()
        rclpy.try_shutdown()
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()

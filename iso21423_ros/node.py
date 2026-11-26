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
from iso21423_ros.nav import NavRunner, runner_kwargs
from iso21423_ros.status import StatusComposer

log = logging.getLogger(__name__)

DEFAULT_EXTENSIONS = {"customData": "iso21423_ros.oro:CustomDataSource"}
RETRY_S = 10
IDLE_WAIT_S = 6.0  # shutdown: how long in-flight requests get to end after their goals are cancelled
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


def _refresh(status):
    """Called outside every lock (lock order: the composer's lock is taken last)."""
    try:
        status.refresh()
    except Exception:
        log.exception("status refresh failed")


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
        self._nav_kw = runner_kwargs(cfg.handlers.get("move", {}))
        self._sources = _enabled("sources", cfg.sources, DEFAULT_SOURCES)
        self._extensions = _enabled("extensions", cfg.extensions, DEFAULT_EXTENSIONS)
        self._handlers = {}
        for name, h in _enabled("handlers", cfg.handlers, DEFAULT_HANDLERS).items():
            if getattr(h[0], "available", lambda: True)():
                self._handlers[name] = h
            else:  # e.g. undock on Humble
                log.warning("handlers.%s: not available on this ROS distro; not accepted", name)
        if not cfg.ccs.id:
            log.warning("ccs.id is empty: locations will carry an empty CCS id")
        self.ccs = CcsFrame(cfg.ccs.id, cfg.ccs.reference_points)
        self.map_frame, self.base_frame = cfg.ccs.map_frame, cfg.base_frame
        self.tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.status = self.nav = self.entity = None
        self.client = None
        self.hooks = []  # sources and extensions
        self.handlers = {}
        self._stop = threading.Event()
        self._lock = threading.Lock()  # guards _closed and the client/entity hand-over only
        self._closed = False
        self._built = threading.Event()  # set once the post-connect build is over (or never started)
        self._built.set()
        self._inflight = 0  # handler executes running (all handlers, vendor ones too)
        self._idle = threading.Condition()

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
            client = None
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
                if client is not None:
                    _guarded([("client close", client.close)])
                self._stop.wait(RETRY_S)
        with self._lock:
            closed = self._closed
            if not closed:
                self.client, self.entity = client, entity
                self._built.clear()  # shutdown() waits for the build before tearing down
        if closed:  # shutdown() ran while we were connecting: it never saw this client
            _guarded([("unregister", entity.unregister), ("client close", client.close)])
            return
        log.info("registered %s on %s: provides %s, accepts %s", cfg.entity_uuid, creds.url, provides, accepts)
        try:
            self._build(entity)
        finally:
            self._built.set()

    def _build(self, entity):
        cfg = self.config
        self.status = StatusComposer(self._publish_status, cfg.mode)
        self.nav = NavRunner(self, **self._nav_kw)
        for cls, params in [*self._sources.values(), *self._extensions.values()]:
            self.hooks.append(cls(self, entity, params))
        for name, (cls, params) in self._handlers.items():
            self.handlers[name] = cls(self, params)
        for h in [*self.hooks, *self.handlers.values()]:
            if callable(getattr(h, "states", None)):
                self.status.add_contributor(h.states)
        self.status.add_contributor(self.nav.states)
        for name, h in self.handlers.items():
            entity.on_request(name, self._tracked(h.execute))
        if self._stop.is_set():  # shutdown() started meanwhile: don't take requests
            return
        entity.start_serving()
        self.status.refresh()

    def _tracked(self, execute):
        def run(action, ctx):
            with self._idle:
                self._inflight += 1
            _refresh(self.status)  # IDLE needs _inflight == 0
            try:
                return execute(action, ctx)
            finally:
                with self._idle:
                    self._inflight -= 1
                    self._idle.notify_all()
                _refresh(self.status)
        return run

    def _publish_status(self, states):
        self.entity.publish_status(states)

    def _details(self):
        ident = self.config.identity
        fp = (self._nav2_footprint() if ident.footprint_from_nav2 else None) or ident.footprint
        if not fp:
            log.warning("identity.footprint is empty: imrFootprint and imrWorkingArea will be empty")
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
        """Idempotent. Run it while the executor still spins (goal cancels must go out):
        end in-flight requests (each cancels its own goal), flush sources, unregister (OFFLINE
        tombstone), close the client. A connect still in progress cleans up after itself."""
        self._stop.set()
        with self._lock:
            if self._closed:
                return
            self._closed = True
        if not self._built.wait(15):  # start_serving() can wait up to 10 s for its SUBACK
            log.warning("shutdown: startup build still running")
        steps = []
        try:
            closers = [self.nav, *self.handlers.values()]
            steps = [(f"close {type(h).__name__}", h.close) for h in closers if callable(getattr(h, "close", None))]
        except Exception:
            log.exception("shutdown: collecting handlers to close failed")
        if self.entity:  # no new requests from here on
            steps.append(("stop serving", self.entity.stop_serving))
        t0 = time.monotonic()
        steps.append(("wait for in-flight requests", self._wait_idle))
        if self.entity:  # the library publishes terminal statuses on its own threads, after handlers return
            steps.append(("wait for library terminal statuses", lambda: self._wait_library_idle(t0)))
        steps.append(("destroy subscriptions/timers", self._destroy_entities))
        try:
            steps += [(f"close {type(h).__name__}", h.close) for h in self.hooks if callable(getattr(h, "close", None))]
        except Exception:
            log.exception("shutdown: collecting sources to close failed")
        if self.status:
            steps.append(("close status", self.status.close))
        if self.entity:
            steps.append(("unregister", self.entity.unregister))
        if self.client:
            steps.append(("client close", self.client.close))
        _guarded(steps)

    def _wait_idle(self):
        with self._idle:
            if not self._idle.wait_for(lambda: self._inflight == 0, IDLE_WAIT_S):
                log.warning("shutdown: %d request(s) still running after %.0fs", self._inflight, IDLE_WAIT_S)

    def _wait_library_idle(self, t0):
        timeout = max(0.5, IDLE_WAIT_S - (time.monotonic() - t0))
        if not self.entity.wait_idle(timeout):
            log.warning("shutdown: library still has active requests after %.1fs", timeout)

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
            if node._stop.is_set():  # a side effect of shutting down, not a startup failure
                log.debug("startup ended by shutdown", exc_info=True)
                return
            log.exception("startup failed")
            failed.append(True)
            done.set()

    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spinning = threading.Event()
    spinning.set()

    def spin():
        while spinning.is_set():
            ex.spin_once(timeout_sec=0.1)

    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()
    threading.Thread(target=startup, daemon=True).start()
    try:
        while not done.wait(0.2) and rclpy.ok():
            pass
        node.shutdown()  # while the executor still spins: Nav2 goal cancels must go out
    finally:
        spinning.clear()
        spinner.join()
        node.destroy_node()
        ex.shutdown()
        rclpy.try_shutdown()
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()

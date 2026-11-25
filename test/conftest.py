import contextlib
import logging
import threading

import pytest
from rclpy.executors import MultiThreadedExecutor


class Spinner:
    """Spins a MultiThreadedExecutor on a thread. stop() joins the thread before shutting the executor
    down: Executor.shutdown() racing a running spin() can hang or crash the spin thread."""

    def __init__(self, *nodes, num_threads=None):
        self.ex, self.done = MultiThreadedExecutor(num_threads=num_threads), threading.Event()
        for n in nodes:
            self.ex.add_node(n)
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def _run(self):
        while not self.done.is_set():
            self.ex.spin_once(timeout_sec=0.05)

    def stop(self):
        self.done.set()
        self.t.join()
        self.ex.shutdown()


@pytest.fixture
def logcap():
    """Own capture: colcon's pytest runs with the logging plugin disabled, so caplog is empty."""
    class H(logging.Handler):
        def __init__(self):
            super().__init__()
            self.records = []

        def emit(self, record):
            self.records.append(record)

        @property
        def text(self):
            return "\n".join(r.getMessage() + (str(r.exc_info[1]) if r.exc_info else "")
                             for r in self.records)

        def at_level(self, level):
            return contextlib.nullcontext()

    h = H()
    # launch_testing swaps the package logger for one that doesn't propagate, so attach to each
    # module logger directly (modules must already be imported).
    lgs = [lg for n, lg in list(logging.root.manager.loggerDict.items())
           if n.startswith("iso21423_ros.") and isinstance(lg, logging.Logger)]
    for lg in lgs:
        lg.addHandler(h)
    yield h
    for lg in lgs:
        lg.removeHandler(h)

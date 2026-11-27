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

import contextlib
import logging
import threading
from pathlib import Path

import pytest
from rclpy.executors import MultiThreadedExecutor


SIM_DIR = Path(__file__).parent / "sim"


def _in_sim(p):
    """test/sim and everything under it: host-run sim tests (scripts/sim_test.sh) and their venv."""
    p = Path(str(p))
    return True if p == SIM_DIR or SIM_DIR in p.parents else None


# Not collect_ignore: Jazzy's launch_testing plugin answers pytest_ignore_collect False (tryfirst) for
# every path, which overrides collect_ignore, --ignore and norecursedirs. A conftest hook runs first.
if int(pytest.__version__.split(".")[0]) >= 7:
    @pytest.hookimpl(tryfirst=True)
    def pytest_ignore_collect(collection_path):
        return _in_sim(collection_path)
else:  # pytest 6 (Humble) has only the py.path argument
    @pytest.hookimpl(tryfirst=True)
    def pytest_ignore_collect(path):
        return _in_sim(path)


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

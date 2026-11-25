import contextlib
import logging

import pytest


@pytest.fixture
def logcap():
    # colcon's pytest run disables the logging plugin, so the built-in caplog is empty.
    """Own capture: colcon's pytest runs with the logging plugin disabled."""
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

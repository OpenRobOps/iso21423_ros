"""Status composer: mode first, then contributor states in priority order."""
import logging
import threading
from typing import Callable

from iso21423.constants import OPERATING_MODES

log = logging.getLogger(__name__)

PRIORITY = ("STOP_CATEGORY_0", "STOP_CATEGORY_1", "STOP_CATEGORY_2", "LOST", "BLOCKED",
            "PAUSED", "WAIT_FOR_RESET", "DOCKING", "CHARGING", "LOW_BATTERY", "FORWARD",
            "REVERSE", "STOPPED", "IDLE")


def _check_mode(mode):
    if mode not in OPERATING_MODES:
        raise ValueError(f"invalid mode {mode!r}, expected one of {OPERATING_MODES}")


class StatusComposer:
    def __init__(self, publish: Callable[[list], None], mode: str):
        _check_mode(mode)
        self._publish = publish
        self._mode = mode
        self._contributors = []
        self._last = None  # tuple, swapped atomically after a successful publish
        self._warned = set()
        # ponytail: compute+publish under one RLock keeps published order exact (a publish
        # outside the lock could reorder); a slow broker publish blocks other refreshers.
        self._lock = threading.RLock()

    def add_contributor(self, fn: Callable[[], list]):
        """fn returns a list/tuple of state strings. It must not call refresh()."""
        with self._lock:
            self._contributors.append(fn)

    def set_mode(self, mode: str):
        _check_mode(mode)
        with self._lock:
            self._mode = mode
        self.refresh()

    def states(self) -> list:
        return list(self._last or ())

    def refresh(self):
        with self._lock:
            seen = []
            for fn in self._contributors:
                try:
                    got = fn()
                    if not isinstance(got, (list, tuple)) or not all(isinstance(s, str) for s in got):
                        raise TypeError(f"expected list/tuple of str, got {got!r}")
                except Exception:
                    log.exception("status contributor %r failed", fn)
                    continue
                for s in got:
                    if s.startswith("MODE_"):
                        if s not in self._warned:
                            self._warned.add(s)
                            log.warning("contributor returned %s; mode is set by the composer", s)
                    elif s not in seen:
                        seen.append(s)
            rank = {s: i for i, s in enumerate(PRIORITY)}
            # stable sort: unknown states keep contributor order after known ones
            seen.sort(key=lambda s: rank.get(s, len(PRIORITY)))
            new = tuple([self._mode] + seen)
            if new != self._last:
                try:
                    self._publish(list(new))
                except Exception:
                    log.exception("status publish failed; will retry on next refresh")
                    return
                self._last = new

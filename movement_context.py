"""Shared motion state for vision guardian / coordinator."""

from __future__ import annotations

import threading


class MovementContext:
    """True while a timed move (straight/reverse/turn) is executing."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._moving = False

    @property
    def is_moving(self) -> bool:
        with self._lock:
            return self._moving

    def set_moving(self, value: bool) -> None:
        with self._lock:
            self._moving = value

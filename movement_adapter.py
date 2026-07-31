"""
Movement API used by voice / planner — drives Arduino via SerialDrivetrain.
Method names match `prompts_and_glossary.commands['movement'][*]['command']`.

v2 → v3 changes:
  - straight() and reverse() now call straight_m() / reverse_m() directly.
    The old duration→ticks approximation is gone; distance is encoder-exact.
  - meters_per_second parameter removed (no longer needed).
"""

from __future__ import annotations

import logging
from typing import Any, List, Optional, Tuple

from prompts_and_glossary import commands as glossary_commands

import config
from drivetrain_client import SerialDrivetrain
from movement_context import MovementContext

logger = logging.getLogger(__name__)


class ArduinoMovement:
    """
    Voice-compatible surface: straight, reverse, left, right, stop.
    Optional MovementContext updates for vision guardian.
    """

    def __init__(
        self,
        drivetrain: Optional[SerialDrivetrain] = None,
        ctx: Optional[MovementContext] = None,
    ) -> None:
        self._owns_dt = drivetrain is None
        self._dt = drivetrain or SerialDrivetrain()
        self._ctx = ctx

    def close(self) -> None:
        if self._owns_dt:
            self._dt.close()

    @property
    def drivetrain(self) -> SerialDrivetrain:
        return self._dt

    def _wrap_moving(self, fn, *args, **kwargs) -> Any:
        if self._ctx:
            self._ctx.set_moving(True)
        try:
            return fn(*args, **kwargs)
        finally:
            if self._ctx:
                self._ctx.set_moving(False)

    def straight(self, meters: Any = None) -> None:
        """Drive forward `meters` metres using encoder-counted movement."""
        m = float(meters) if meters is not None else config.DEFAULT_MOVE_METERS
        m = abs(m)
        logger.info("straight %.3f m (encoder-counted)", m)
        self._wrap_moving(self._dt.straight_m, meters=m)

    def reverse(self, meters: Any = None) -> None:
        """Drive backward `meters` metres using encoder-counted movement."""
        m = float(meters) if meters is not None else config.DEFAULT_MOVE_METERS
        m = abs(m)
        logger.info("reverse %.3f m (encoder-counted)", m)
        self._wrap_moving(self._dt.reverse_m, meters=m)

    def left(self, angle: Any = None) -> None:
        deg = float(angle) if angle is not None else config.DEFAULT_TURN_DEGREES
        logger.info("left %.1f deg", deg)
        self._wrap_moving(self._dt.left, angle=deg)

    def right(self, angle: Any = None) -> None:
        deg = float(angle) if angle is not None else config.DEFAULT_TURN_DEGREES
        logger.info("right %.1f deg", deg)
        self._wrap_moving(self._dt.right, angle=deg)

    def stop(self, _unused: Any = None) -> None:
        logger.info("stop")
        self._dt.stop()


class MovementHistory:
    """Tracks voice movement steps for origin / backtrack."""

    def __init__(self, move: ArduinoMovement) -> None:
        self._move = move
        self._stack: List[Tuple[str, Any]] = []

    @property
    def stack(self) -> List[Tuple[str, Any]]:
        return list(self._stack)

    def clear(self) -> None:
        self._stack.clear()

    def apply_voice_word(self, word: str, value: Any) -> None:
        meta = glossary_commands["movement"].get(word)
        if not meta:
            raise ValueError(f"unknown movement token: {word}")
        cmd = meta["command"]
        method = getattr(self._move, cmd)
        if cmd == "stop":
            method()
            return
        method(value)
        self._stack.append((word, value))

    def origin(self, steps: Any) -> None:
        n = int(steps) if steps is not None else -1
        if n == -1:
            while self._stack:
                self._undo_one()
        else:
            for _ in range(min(n, len(self._stack))):
                self._undo_one()

    def _undo_one(self) -> None:
        if not self._stack:
            return
        word, value = self._stack.pop()
        comp = glossary_commands["movement"][word]["complement"]
        if comp is None:
            return
        getattr(self._move, comp)(value)

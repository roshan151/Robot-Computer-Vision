"""
Movement API over the Arduino drivetrain.

Thin by design: it names the four things the robot can do, tracks the moving
flag for the vision guardian, and exposes the out-of-band brake. Sequencing,
queueing and cancellation all live in MotionExecutor.
"""

from __future__ import annotations

import logging
from typing import Any, Optional

import config
from drivetrain import SerialDrivetrain
from movement_context import MovementContext

logger = logging.getLogger(__name__)


class ArduinoMovement:
    """straight / reverse / left / right, plus two ways to stop."""

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
        """Drive forward `meters` metres, encoder-counted. Blocks."""
        m = abs(float(meters)) if meters is not None else config.DEFAULT_MOVE_METERS
        logger.info("straight %.3f m", m)
        self._wrap_moving(self._dt.straight_m, meters=m)

    def reverse(self, meters: Any = None) -> None:
        """Drive backward `meters` metres, encoder-counted. Blocks."""
        m = abs(float(meters)) if meters is not None else config.DEFAULT_MOVE_METERS
        logger.info("reverse %.3f m", m)
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
        """Graceful stop.

        WARNING: blocks behind any in-flight encoder-counted move — the bridge
        holds _cmd_lock for the whole move, so this arrives only after the move
        it was meant to interrupt has finished. Use emergency_stop() to
        actually interrupt one.
        """
        logger.info("stop")
        self._dt.stop()

    def emergency_stop(self) -> None:
        """Halt a move already in progress. Safe from any thread.

        Writes the S frame out of band, bypassing the command lock the running
        move is holding. This is the only stop that works mid-move.

        Deliberately does not log — every caller already emits an `estop` event
        with the reason attached, and the reason is the useful half.
        """
        self._dt.emergency_stop()
        if self._ctx:
            self._ctx.set_moving(False)

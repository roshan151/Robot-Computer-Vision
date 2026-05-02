"""
Background vision → stop policy while the robot is executing a timed move.
"""

from __future__ import annotations

import logging
import threading
from typing import List, Optional

import config
from movement_adapter import ArduinoMovement
from movement_context import MovementContext
from vision_client import RobotVision

logger = logging.getLogger(__name__)


class VisionGuardian(threading.Thread):
    """
    If VISION_HALT_OBJECTS is non-empty and any are seen while `ctx.is_moving`,
    send stop to Arduino (voice / planner can resume later).
    """

    def __init__(
        self,
        vision: RobotVision,
        move: ArduinoMovement,
        ctx: MovementContext,
        objects: Optional[List[str]] = None,
        hz: float = config.VISION_GUARD_HZ,
    ) -> None:
        super().__init__(daemon=True)
        self._vision = vision
        self._move = move
        self._ctx = ctx
        self._objects = objects or list(config.VISION_HALT_OBJECTS)
        self._hz = max(0.5, hz)
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        if not self._objects:
            logger.info("VisionGuardian: no VISION_HALT_OBJECTS; idle")
            while not self._stop.is_set():
                self._stop.wait(1.0)
            return

        period = 1.0 / self._hz
        logger.info("VisionGuardian: watching for %s", self._objects)
        while not self._stop.is_set():
            if self._stop.wait(period):
                break
            if not self._ctx.is_moving:
                continue
            try:
                if self._vision.detect_objects(self._objects):
                    logger.warning("guardian: halt condition detected → stop")
                    self._move.stop()
            except Exception as e:
                logger.debug("guardian tick failed: %s", e)


class RobotCoordinator:
    def __init__(
        self,
        move: ArduinoMovement,
        vision: RobotVision,
        ctx: MovementContext,
        halt_objects: Optional[List[str]] = None,
    ) -> None:
        self._guardian = VisionGuardian(vision, move, ctx, objects=halt_objects)

    def start_guardian(self) -> None:
        self._guardian.start()

    def stop_guardian(self) -> None:
        self._guardian.stop()
        self._guardian.join(timeout=2.0)

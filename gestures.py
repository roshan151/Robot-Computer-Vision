"""
The robot's output channel.

It never speaks during normal operation — audio is reserved for the failure path
and only plays once the voice session is already torn down, which is what keeps
the microphone free of the robot's own voice.  So everything the robot has to
say, it says by moving.

Vocabulary
----------
    YES      forward 0.1 m, back 0.1 m          (a nod)
    NO       left 30°, right 60°, left 30°      (a head shake, net zero)
    UNCLEAR  same as NO — "no" and "I didn't understand you" are deliberately
             the same gesture

Thinking has no gesture, by design.  Note the consequence: absence of motion is
ambiguous between *thinking*, *no*, and *the session is dead* — a silent robot
cannot be told apart from a broken one without reading logs.json.

Gestures are net-zero on paper and not in practice — wheel slip on three
consecutive turns accumulates heading error.  That is accepted; this build does
not depend on odometry.
"""

from __future__ import annotations

import logging
from typing import List, Optional, Tuple

import config
import robot_log
from motion_executor import MotionExecutor

logger = logging.getLogger(__name__)

Step = Tuple[str, float]

YES: List[Step] = [
    ("straight", config.GESTURE_YES_METERS),
    ("reverse", config.GESTURE_YES_METERS),
]

NO: List[Step] = [
    ("left", config.GESTURE_NO_DEGREES),
    ("right", config.GESTURE_NO_DEGREES * 2),
    ("left", config.GESTURE_NO_DEGREES),
]

UNCLEAR: List[Step] = NO

VOCABULARY = {
    "yes": YES,
    "no": NO,
    "unclear": UNCLEAR,
}


class Gesturer:
    """Plays gestures without ever blocking the caller."""

    def __init__(self, executor: MotionExecutor) -> None:
        self._exec = executor

    def play(self, name: str) -> Optional[List[int]]:
        """Queue a gesture. Returns job ids, or None if it was suppressed.

        Two suppression rules, both deliberate:

        1. If a gesture is already running, drop this one. Queueing turns a
           burst of answers into a twenty-second dance that outlives the
           conversation that prompted it.
        2. If a real (non-gesture) move is running, skip entirely. The robot
           should not nod while driving somewhere — motion is both the actuator
           and the display, and the actuator wins.
        """
        key = str(name).lower()
        steps = VOCABULARY.get(key)
        if steps is None:
            raise ValueError(
                f"unknown gesture {name!r}; expected one of {sorted(VOCABULARY)}"
            )

        status = self._exec.status()
        if status["moving"]:
            robot_log.event(
                "gesture.skip", name=key,
                why="gesturing" if status["gesturing"] else "driving",
            )
            return None
        if status["queue_depth"]:
            robot_log.event("gesture.skip", name=key, why="queue busy",
                            depth=status["queue_depth"])
            return None

        jobs = self._exec.submit_sequence(steps, gesture=True)
        robot_log.event("gesture", name=key, jobs=jobs)
        return jobs

    def yes(self) -> Optional[List[int]]:
        return self.play("yes")

    def no(self) -> Optional[List[int]]:
        return self.play("no")

    def unclear(self) -> Optional[List[int]]:
        return self.play("unclear")

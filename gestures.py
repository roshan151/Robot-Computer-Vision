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
ambiguous between *thinking*, *no*, and *the session is dead*.  Phase 4's phone
UI heartbeat is what resolves that, and until it exists a silent robot cannot be
distinguished from a broken one.

Gestures are net-zero on paper and not in practice — wheel slip on three
consecutive turns accumulates heading error.  That is accepted; this build does
not depend on odometry.  They are still kept out of MovementHistory so that
`origin()` never tries to replay a conversation backwards.
"""

from __future__ import annotations

import logging
from typing import Any, List, Optional, Tuple

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

# Gesture op -> ArduinoMovement method. "forward" is the planner's word;
# the drivetrain calls it "straight".
_MOVE_METHOD = {
    "straight": "straight",
    "forward": "straight",
    "reverse": "reverse",
    "left": "left",
    "right": "right",
}


class SyncGesturer:
    """Plays a gesture on the calling thread, blocking until it finishes.

    For the turn-based voice loop, where a gesture is the robot's entire reply
    and there is nothing else to do while it plays.

    Safe without the executor because of how that loop is shaped: planner steps
    already run blocking on the main thread via `_dispatch_step`, and the
    planner never returns an `answer` and `steps` in the same turn — so a
    gesture and a movement can never overlap. Same thread, same
    ArduinoMovement, strictly sequential, no second owner of the serial link.

    Gestures still bypass MovementHistory. `origin()` walks that stack to
    backtrack; conversational nods in it would make "return to origin" replay
    the conversation.

    When the MotionExecutor is wired into the session (Phase 2), swap this for
    `Gesturer` — same `play()` signature, same vocabulary.
    """

    def __init__(self, move: Any) -> None:
        self._move = move

    def play(self, name: str) -> bool:
        key = str(name).lower()
        steps = VOCABULARY.get(key)
        if steps is None:
            raise ValueError(
                f"unknown gesture {name!r}; expected one of {sorted(VOCABULARY)}"
            )

        robot_log.event("gesture", name=key, mode="sync", steps=len(steps))
        for op, value in steps:
            try:
                getattr(self._move, _MOVE_METHOD[op])(value)
            except Exception as e:
                # A failed gesture must not take the voice loop down with it —
                # the operator simply gets no answer, which the log explains.
                robot_log.event("gesture.skip", logging.WARNING, name=key,
                                why=f"step {op} failed",
                                err=f"{type(e).__name__}: {e}")
                return False
        return True

    def yes(self) -> bool:
        return self.play("yes")

    def no(self) -> bool:
        return self.play("no")

    def unclear(self) -> bool:
        return self.play("unclear")


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

"""
The robot's functions, as Gemini Live tools.

Every tool returns IMMEDIATELY
------------------------------
This is the load-bearing property, and it is a correctness requirement rather
than a nicety.

Tool calls are dispatched from inside `async for response in session.receive()`.
Awaiting a 1.4 s drive inside that loop stops the websocket being read for 1.4 s.
The receive side stops draining, back-pressure reaches the send side, the
microphone uplink stalls, audio starts being dropped, and the session dies —
which is exactly the failure we saw: two turns, `audio.error stage=mic-queue`,
then `voice.drop`, then a reconnect that survives one more command.

So no tool blocks. Moves are queued on a MotionExecutor — a FIFO worker thread
that owns the drivetrain — and the tool answers "queued" straight away. The
event loop is then free to keep reading the socket and pumping the microphone,
which is what "always listening" actually requires.

`stop` is the exception, deliberately: it runs inline, empties the queue, and
fires the out-of-band brake. A stop that waits its turn behind the moves it is
meant to cancel is not a stop.
"""

from __future__ import annotations

import logging
from typing import Any, Callable, Dict, Optional

import config
import robot_log
from gestures import Gesturer
from motion_executor import MotionExecutor
from movement_adapter import ArduinoMovement

logger = logging.getLogger(__name__)


def declarations() -> list:
    """FunctionDeclarations for the Live config.

    Deliberately small. Every tool is context the model re-reads on every turn,
    and a robot that can do four things reliably beats one that can nominally
    do ten.
    """
    from google.genai import types  # type: ignore

    def schema(**props):
        required = [k for k, v in props.items() if v.pop("_required", False)]
        return types.Schema(
            type="OBJECT",
            properties={k: types.Schema(**v) for k, v in props.items()},
            required=required,
        )

    return [
        types.FunctionDeclaration(
            name="drive",
            description=(
                "Queue a straight move. Positive metres go forward, negative "
                "go backward. Returns as soon as the move is queued, not when "
                "it finishes — you can keep listening and queue more."
            ),
            parameters=schema(meters={
                "type": "NUMBER",
                "description": "Distance in metres. Negative to reverse.",
                "_required": True,
            }),
        ),
        types.FunctionDeclaration(
            name="turn",
            description=(
                "Queue a turn on the spot. Positive degrees turn RIGHT, "
                "negative turn LEFT. Returns as soon as it is queued."
            ),
            parameters=schema(degrees={
                "type": "NUMBER",
                "description": "Degrees to turn. Negative for left.",
                "_required": True,
            }),
        ),
        types.FunctionDeclaration(
            name="stop",
            description=(
                "Stop immediately: cancels everything queued AND interrupts "
                "the move in progress. Call this the instant the operator says "
                "stop, and whenever you are unsure whether it is safe to keep "
                "moving."
            ),
            parameters=schema(),
        ),
        types.FunctionDeclaration(
            name="answer",
            description=(
                "Answer a yes/no question by gesturing, since the robot has no "
                "voice. 'yes' nods; 'no' shakes; 'unclear' uses the same shake "
                "as 'no' and means you could not make out the speech."
            ),
            parameters=schema(value={
                "type": "STRING",
                "description": "One of: yes, no, unclear",
                "_required": True,
            }),
        ),
    ]


class RobotTools:
    """Executes the model's tool calls. Nothing here ever blocks the loop."""

    def __init__(self, move: ArduinoMovement) -> None:
        self._move = move
        # FIFO, single worker thread, sole owner of the serial link. submit()
        # returns a job id without waiting, cancel_all() empties the queue and
        # brakes whatever is already running.
        self._exec = MotionExecutor(move).start()
        self._gestures = Gesturer(self._exec)

    def close(self) -> None:
        self._exec.close()

    @property
    def executor(self) -> MotionExecutor:
        return self._exec

    # ------------------------------------------------------------------ #

    async def dispatch(self, name: str, args: Dict[str, Any]) -> Dict[str, Any]:
        """Run one tool call. Never raises — a tool error is a result.

        Not actually async work: every handler is synchronous and fast. The
        coroutine signature is kept so the caller can await it uniformly.
        """
        handler: Optional[Callable[..., Dict[str, Any]]] = {
            "drive": self._drive,
            "turn": self._turn,
            "stop": self._stop,
            "answer": self._answer,
        }.get(name)

        if handler is None:
            robot_log.event("voice.tool", logging.WARNING, name=name,
                            ok=False, err="unknown tool")
            return {"ok": False, "error": f"unknown tool {name!r}"}

        try:
            result = handler(**args)
            robot_log.event("voice.tool", name=name, args=args, **result)
            return result
        except TypeError as e:
            robot_log.event("voice.tool", logging.WARNING, name=name,
                            args=args, ok=False, err=str(e))
            return {"ok": False, "error": f"bad arguments: {e}"}
        except Exception as e:
            robot_log.event("voice.tool", logging.ERROR, name=name, args=args,
                            ok=False, err=f"{type(e).__name__}: {e}")
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}

    # ------------------------------------------------------------------ #

    def _queued(self, job_id: int, **extra: Any) -> Dict[str, Any]:
        st = self._exec.status()
        return {"ok": True, "queued": job_id,
                "queue_depth": st["queue_depth"], **extra}

    def _drive(self, meters: float) -> Dict[str, Any]:
        m = float(meters)
        if abs(m) > config.MAX_DRIVE_METERS:
            # Clamped here rather than in the prompt: a limit the model can
            # talk itself out of is not a limit.
            return {"ok": False,
                    "error": f"refused: {m} m exceeds the "
                             f"{config.MAX_DRIVE_METERS} m limit"}
        if abs(m) < 1e-3:
            return {"ok": True, "note": "zero distance, nothing to do"}

        op = "straight" if m > 0 else "reverse"
        return self._queued(self._exec.submit(op, abs(m)), meters=m)

    def _turn(self, degrees: float) -> Dict[str, Any]:
        d = float(degrees)
        if abs(d) < 0.5:
            return {"ok": True, "note": "zero angle, nothing to do"}
        op = "right" if d > 0 else "left"
        return self._queued(self._exec.submit(op, abs(d)), degrees=d)

    def _stop(self) -> Dict[str, Any]:
        """Empty the queue and interrupt the move already running.

        Inline and out of band. cancel_all() clears every pending job, sets the
        cancel flag, and writes the S frame straight to the serial port —
        bypassing the command lock that the running move is holding. Anything
        that waited its turn here would arrive after the move it was meant to
        cancel had already finished.
        """
        dropped = self._exec.cancel_all("voice: stop")
        return {"ok": True, "stopped": True, "cancelled": dropped}

    def _answer(self, value: str) -> Dict[str, Any]:
        v = str(value).lower().strip()
        if v not in ("yes", "no", "unclear"):
            return {"ok": False, "error": f"unknown answer {value!r}"}
        # Gesturer drops the gesture if the robot is already moving: motion is
        # both the actuator and the display, and the actuator wins.
        jobs = self._gestures.play(v)
        if jobs is None:
            return {"ok": True, "gestured": False, "note": "busy moving"}
        return {"ok": True, "gestured": True, "value": v, "jobs": jobs}

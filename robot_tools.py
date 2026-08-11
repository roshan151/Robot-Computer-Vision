"""
The robot's functions, as Gemini Live tools.

The turn-based planner asked the model for a JSON plan and then interpreted it.
The Live agent instead gives the model the robot's actual capabilities and lets
it call them. The schema is no longer a contract we parse — it is the function
signature itself.

Concurrency
-----------
The Live session runs on an asyncio event loop; the drivetrain is blocking and
speaks over a serial link. Every move therefore runs in a worker thread, so a
two-second drive cannot stall the audio stream — if it did, the microphone
would stop feeding the model mid-command.

`stop` is the exception, deliberately. It runs inline on the event loop and
uses the out-of-band emergency path, because a stop that queues behind the move
it is meant to interrupt is not a stop. Everything else can wait; this cannot.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any, Awaitable, Callable, Dict, Optional

import config
import robot_log
from gestures import SyncGesturer
from movement_adapter import ArduinoMovement, MovementHistory

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
                "Drive the robot straight. Positive metres go forward, "
                "negative go backward. Use this for any forward/back movement."
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
                "Turn the robot on the spot. Positive degrees turn RIGHT, "
                "negative turn LEFT."
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
                "Stop all motion immediately, interrupting any move in "
                "progress. Call this the instant the operator says stop, and "
                "whenever you are unsure whether it is safe to keep moving."
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
    """Executes the model's tool calls against the real drivetrain."""

    def __init__(
        self,
        move: ArduinoMovement,
        history: Optional[MovementHistory] = None,
        loop: Optional[asyncio.AbstractEventLoop] = None,
    ) -> None:
        self._move = move
        self._history = history
        self._gestures = SyncGesturer(move)
        self._loop = loop
        # One at a time: the serial link has a single command lock, and two
        # concurrent moves would simply queue anyway — but queued behind a
        # lock the stop path also needs.
        self._motion = asyncio.Semaphore(1)
        self._busy = False

    @property
    def busy(self) -> bool:
        return self._busy

    # ------------------------------------------------------------------ #

    async def dispatch(self, name: str, args: Dict[str, Any]) -> Dict[str, Any]:
        """Run one tool call. Never raises — a tool error is a result."""
        handler: Optional[Callable[..., Awaitable[Dict[str, Any]]]] = {
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
            result = await handler(**args)
            robot_log.event("voice.tool", name=name, args=args, **result)
            return result
        except TypeError as e:
            # Wrong or missing arguments from the model.
            robot_log.event("voice.tool", logging.WARNING, name=name,
                            args=args, ok=False, err=str(e))
            return {"ok": False, "error": f"bad arguments: {e}"}
        except Exception as e:
            robot_log.event("voice.tool", logging.ERROR, name=name, args=args,
                            ok=False, err=f"{type(e).__name__}: {e}")
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}

    # ------------------------------------------------------------------ #

    async def _run_blocking(self, fn, *args) -> None:
        """Move the blocking serial call off the event loop.

        Without this a 2 s drive would stop the microphone task from running,
        and the model would simply stop hearing the operator mid-command.
        """
        async with self._motion:
            self._busy = True
            try:
                await asyncio.to_thread(fn, *args)
            finally:
                self._busy = False

    async def _drive(self, meters: float) -> Dict[str, Any]:
        m = float(meters)
        if abs(m) > config.MAX_DRIVE_METERS:
            # Clamped here rather than in the prompt: a limit the model can
            # talk itself out of is not a limit.
            return {"ok": False,
                    "error": f"refused: {m} m exceeds the "
                             f"{config.MAX_DRIVE_METERS} m limit"}
        if abs(m) < 1e-3:
            return {"ok": True, "note": "zero distance, nothing to do"}

        fn = self._move.straight if m > 0 else self._move.reverse
        await self._run_blocking(fn, abs(m))
        if self._history is not None:
            self._history._stack.append(
                ("straight" if m > 0 else "reverse", abs(m)))
        return {"ok": True, "moved_m": m}

    async def _turn(self, degrees: float) -> Dict[str, Any]:
        d = float(degrees)
        if abs(d) < 0.5:
            return {"ok": True, "note": "zero angle, nothing to do"}
        fn = self._move.right if d > 0 else self._move.left
        await self._run_blocking(fn, abs(d))
        if self._history is not None:
            self._history._stack.append(("right" if d > 0 else "left", abs(d)))
        return {"ok": True, "turned_deg": d}

    async def _stop(self) -> Dict[str, Any]:
        """Inline and out of band — see the module docstring.

        This does NOT take the motion semaphore and does NOT go to a worker
        thread. Both would make it wait for the move it exists to interrupt.
        """
        self._move.emergency_stop()
        robot_log.event("estop", logging.WARNING, reason="voice: stop tool")
        return {"ok": True, "stopped": True}

    async def _answer(self, value: str) -> Dict[str, Any]:
        v = str(value).lower().strip()
        if v not in ("yes", "no", "unclear"):
            return {"ok": False, "error": f"unknown answer {value!r}"}
        if self._busy:
            # Motion is both the actuator and the display; the actuator wins.
            return {"ok": True, "gestured": False, "note": "busy driving"}
        await asyncio.to_thread(self._gestures.play, v)
        return {"ok": True, "gestured": True, "value": v}

"""
Intent-level drivetrain API on the Raspberry Pi — v2.

Uses encoder-counted moves (M: command) instead of time.sleep().
The Arduino signals completion; the Pi just blocks until done.
"""

from __future__ import annotations

import logging
import threading
from typing import Any, Dict

from arduino_bridge import ArduinoBridge
import config

logger = logging.getLogger(__name__)


def duty_from_percent(speed_percent: float) -> int:
    """Map 0-100 style speed to 0-255 PWM."""
    return max(0, min(255, int(round(speed_percent / 100.0 * 255.0))))


# ── Calibration constants ──────────────────────────────────────────────────
# Tune these once for your robot on a flat surface.

# Encoder ticks per centimetre of straight travel.
# Measure: command 100 ticks, measure actual distance, adjust.
TICKS_PER_CM: float = 20.0

# Encoder ticks per degree of tank-turn.
# Measure: command 360 ticks, count actual degrees, adjust.
TICKS_PER_DEGREE: float = 3.5


class SerialDrivetrain:
    def __init__(
        self,
        port: str = config.SERIAL_PORT,
        baud: int = config.BAUD_RATE,
    ) -> None:
        self._enc_lock    = threading.Lock()
        self._motor1_count = 0
        self._motor2_count = 0

        def on_enc(left: int, right: int) -> None:
            with self._enc_lock:
                self._motor1_count = left
                self._motor2_count = right

        def on_err() -> None:
            logger.error("Arduino reported ERR")

        self._bridge = ArduinoBridge(
            port=port,
            baud=baud,
            on_encoder=on_enc,
            on_error=on_err,
        )

    def close(self) -> None:
        self._bridge.close()

    # ------------------------------------------------------------------ #
    # High-level movement API  (encoder-counted, blocking)
    # ------------------------------------------------------------------ #

    def straight(self, duration: float = 3.0, speed: float = 70.0) -> None:
        """
        Drive forward for `duration` seconds worth of distance.

        Internally converted to encoder ticks so the Arduino auto-stops.
        `duration` is kept as the public parameter for backwards compatibility
        with test_bot_movements.py, but the move is tick-controlled, not timed.

        If you prefer to specify distance directly, call straight_cm() instead.
        """
        # Approximate ticks from duration: ticks ≈ speed_pct * scale * duration
        # This keeps the existing test script working without changes.
        ticks = _duration_to_ticks(duration, speed)
        self._bridge.move("F", duty_from_percent(speed), ticks)

    def straight_cm(self, cm: float, speed: float = 70.0) -> None:
        """Drive forward exactly `cm` centimetres (requires TICKS_PER_CM calibration)."""
        ticks = max(1, int(cm * TICKS_PER_CM))
        self._bridge.move("F", duty_from_percent(speed), ticks)

    def reverse(self, duration: float = 3.0, speed: float = 70.0) -> None:
        ticks = _duration_to_ticks(duration, speed)
        self._bridge.move("B", duty_from_percent(speed), ticks)

    def reverse_cm(self, cm: float, speed: float = 70.0) -> None:
        ticks = max(1, int(cm * TICKS_PER_CM))
        self._bridge.move("B", duty_from_percent(speed), ticks)

    def right(self, angle: float = 90.0, speed: float = 50.0) -> None:
        ticks = max(1, int(angle * TICKS_PER_DEGREE))
        self._bridge.move("R", duty_from_percent(speed), ticks)

    def left(self, angle: float = 90.0, speed: float = 50.0) -> None:
        ticks = max(1, int(angle * TICKS_PER_DEGREE))
        self._bridge.move("L", duty_from_percent(speed), ticks)

    def stop(self) -> None:
        self._bridge.stop()

    # ------------------------------------------------------------------ #
    # Open-loop intent methods (fire-and-forget, for manual/streaming use)
    # ------------------------------------------------------------------ #

    def intent_forward(self)  -> None: self._bridge.forward()
    def intent_backward(self) -> None: self._bridge.backward()
    def intent_left(self)     -> None: self._bridge.left()
    def intent_right(self)    -> None: self._bridge.right()

    # ------------------------------------------------------------------ #
    # Encoder status
    # ------------------------------------------------------------------ #

    def get_encoder_status(self) -> Dict[str, Any]:
        with self._enc_lock:
            c1 = self._motor1_count
            c2 = self._motor2_count
        return {
            "motor1_count": c1,
            "motor2_count": c2,
            "sync_error":   c1 - c2,
        }


# ── Private helpers ────────────────────────────────────────────────────────

def _duration_to_ticks(duration_s: float, speed_pct: float) -> int:
    """
    Rough tick estimate from a duration + speed.

    Uses TICKS_PER_CM as the baseline so the scale is consistent with
    straight_cm().  Tune TICKS_PER_CM so that:
        straight(duration=1.0, speed=100.0) travels ~(100 * _SPEED_SCALE) cm.
    """
    _SPEED_SCALE = 0.5   # cm/s at 100% speed — adjust after calibration
    cm = duration_s * (speed_pct / 100.0) * _SPEED_SCALE * 100.0
    return max(1, int(cm * TICKS_PER_CM))
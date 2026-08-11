"""
Intent-level drivetrain API on the Raspberry Pi — v3.
Key changes vs v2:
  - Public straight/reverse API now takes METERS, not duration seconds.
    Internally converted to encoder ticks via config.TICKS_PER_CM so the
    Arduino auto-stops when the target distance is reached — no time.sleep().
  - Duration-based straight()/reverse() removed. All straight moves are
    encoder-controlled end-to-end.
  - _verify_encoder_delta() runs after every encoder-counted move and logs
    warnings for stalled wheels or excessive sync error.
  - Calibration constants (TICKS_PER_CM, TICKS_PER_DEGREE) live in config.py
    so they can be tuned via environment variables without touching code.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, Optional

from .arduino_bridge import ArduinoBridge
import config

logger = logging.getLogger(__name__)


def duty_from_percent(speed_percent: float) -> int:
    """Map 0–100 % speed to 0–255 PWM duty cycle."""
    return max(0, min(255, int(round(speed_percent / 100.0 * 255.0))))


def _speed(speed: Optional[float]) -> float:
    """Resolve a speed argument against the configured default.

    Ramping is handled by the firmware (RAMP_STEP/RAMP_MS in drivetrain.ino,
    ~0.5 PWM per ms up, 2x that on softStop), which is what keeps the inrush
    and back-EMF off the supply rail.  Nothing here should ramp — a second
    ramp on the host would only fight it.
    """
    return config.DEFAULT_SPEED_PERCENT if speed is None else float(speed)


def _ticks_per_metre() -> float:
    """Encoder ticks per metre, derived from config.TICKS_PER_CM."""
    return config.TICKS_PER_CM * 100.0


class SerialDrivetrain:
    def __init__(
        self,
        port: str = config.SERIAL_PORT,
        baud: int = config.BAUD_RATE,
    ) -> None:
        self._enc_lock     = threading.Lock()
        self._motor1_count = 0
        self._motor2_count = 0

        # Default base speed used by intent (open-loop) commands.
        self._base_speed_pct: float = 60.0

        def on_enc(left: int, right: int) -> None:
            with self._enc_lock:
                self._motor1_count = left
                self._motor2_count = right

        def on_err() -> None:
            logger.error("Arduino reported ERR (unsolicited)")

        self._bridge = ArduinoBridge(
            port=port,
            baud=baud,
            on_encoder=on_enc,
            on_error=on_err,
        )

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def close(self) -> None:
        self._bridge.close()

    @property
    def is_connected(self) -> bool:
        return self._bridge.is_connected

    # ------------------------------------------------------------------ #
    # High-level movement API  (encoder-counted, blocking, in metres)
    # ------------------------------------------------------------------ #

    def straight_m(self, meters: float, speed: Optional[float] = None) -> None:
        """
        Drive forward exactly `meters` metres.

        Uses encoder ticks — the Arduino stops when the tick target is
        reached regardless of battery voltage or terrain friction.
        Logs a warning if either wheel stalls or if left/right drift
        exceeds config.ENCODER_SYNC_WARN_RATIO.

        Args:
            meters: Distance in metres (must be > 0).
            speed:  Speed as 0–100 % of full PWM.
                    None -> config.DEFAULT_SPEED_PERCENT.
        """
        if meters <= 0:
            raise ValueError(f"straight_m: meters must be positive, got {meters}")
        ticks = max(1, int(meters * _ticks_per_metre()))
        self._move_and_verify(
            "F", duty_from_percent(_speed(speed)), ticks,
            label=f"straight_m({meters:.3f}m)",
        )

    def reverse_m(self, meters: float, speed: Optional[float] = None) -> None:
        """
        Drive backward exactly `meters` metres (encoder-counted).

        Args:
            meters: Distance in metres (must be > 0).
            speed:  Speed as 0–100 %. None -> config.DEFAULT_SPEED_PERCENT.
        """
        if meters <= 0:
            raise ValueError(f"reverse_m: meters must be positive, got {meters}")
        ticks = max(1, int(meters * _ticks_per_metre()))
        self._move_and_verify(
            "B", duty_from_percent(_speed(speed)), ticks,
            label=f"reverse_m({meters:.3f}m)",
        )

    def right(self, angle: float = 90.0, speed: Optional[float] = None) -> None:
        """Tank-turn right by `angle` degrees (encoder-counted)."""
        if angle <= 0:
            raise ValueError(f"right: angle must be positive, got {angle}")
        ticks = max(1, int(angle * config.TICKS_PER_DEGREE))
        self._move_and_verify(
            "R", duty_from_percent(_speed(speed)), ticks,
            label=f"right({angle:.1f}°)",
        )

    def left(self, angle: float = 90.0, speed: Optional[float] = None) -> None:
        """Tank-turn left by `angle` degrees (encoder-counted)."""
        if angle <= 0:
            raise ValueError(f"left: angle must be positive, got {angle}")
        ticks = max(1, int(angle * config.TICKS_PER_DEGREE))
        self._move_and_verify(
            "L", duty_from_percent(_speed(speed)), ticks,
            label=f"left({angle:.1f}°)",
        )

    def stop(self) -> None:
        """Graceful stop. BLOCKS behind any in-flight move() — see
        emergency_stop() for the path that can interrupt one."""
        self._bridge.stop()

    def emergency_stop(self) -> None:
        """Halt a move already in progress. Safe from any thread."""
        self._bridge.emergency_stop()

    # ------------------------------------------------------------------ #
    # Open-loop intent methods (fire-and-forget / brain_loop streaming use)
    # ------------------------------------------------------------------ #

    def intent_forward(self)  -> None: self._bridge.forward()
    def intent_backward(self) -> None: self._bridge.backward()
    def intent_left(self)     -> None: self._bridge.left()
    def intent_right(self)    -> None: self._bridge.right()

    def set_base_speed_percent(self, pct: float) -> None:
        """Set the V: speed register on the Arduino (for open-loop intent commands)."""
        self._base_speed_pct = max(0.0, min(100.0, pct))
        self._bridge.set_speed_pwm(duty_from_percent(self._base_speed_pct))

    # ------------------------------------------------------------------ #
    # Encoder status
    # ------------------------------------------------------------------ #

    def get_encoder_status(self) -> Dict[str, Any]:
        """Return the latest encoder counts and left-right sync error."""
        with self._enc_lock:
            c1 = self._motor1_count
            c2 = self._motor2_count
        return {
            "motor1_count": c1,
            "motor2_count": c2,
            "sync_error":   c1 - c2,
        }

    # ------------------------------------------------------------------ #
    # Internal helpers
    # ------------------------------------------------------------------ #

    def _move_and_verify(
        self,
        direction: str,
        speed_pwm: int,
        ticks: int,
        label: str,
    ) -> None:
        """
        Execute an encoder-counted move and verify the result.

        Logs warnings (never raises) for:
          - A wheel that didn't move at all (wiring / encoder fault)
          - Left-right sync error above config.ENCODER_SYNC_WARN_RATIO
          - Actual travel significantly below the tick target (stall/slip)
        """
        logger.debug("%s → M:%s,%d,%d", label, direction, speed_pwm, ticks)

        self._bridge.move(direction, speed_pwm, ticks)

        # The firmware zeroes BOTH counters in beginDrive() at the start of
        # every encoder-counted move, so the counts we read now already are
        # this move's travel.  Subtracting a "before" snapshot would straddle
        # that reset and produce nonsense (which is why the old reverse rows
        # reported drift figures that had nothing to do with the robot).
        #
        # ENC: telemetry arrives every 100 ms, so wait one interval for the
        # post-stop reading rather than using the last mid-move sample.
        time.sleep(config.ENCODER_SETTLE_S)
        self._verify_encoder_delta(self.get_encoder_status(), ticks, label)

    def move_counts(self) -> Dict[str, int]:
        """Signed encoder counts for the most recent encoder-counted move.

        Valid straight after a straight_m/reverse_m/left/right call, because
        the firmware resets both counters when a move begins.
        """
        status = self.get_encoder_status()
        return {"left": status["motor1_count"], "right": status["motor2_count"]}

    def _verify_encoder_delta(
        self,
        counts: Dict[str, Any],
        expected_ticks: int,
        label: str,
    ) -> None:
        """
        Inspect how much each encoder moved and warn about anomalies.

        This is a diagnostic/safety layer — it does not retry or compensate.
        It surfaces problems early so the operator can tune calibration
        constants or inspect hardware before a mission.
        """
        dL = abs(counts["motor1_count"])
        dR = abs(counts["motor2_count"])

        # ── Stall detection ──────────────────────────────────────────────
        if dL == 0 and dR == 0:
            logger.error(
                "%s: NEITHER encoder moved — "
                "check motor power, encoder wiring, and serial connection",
                label,
            )
            return

        if dL == 0:
            logger.warning(
                "%s: LEFT encoder did not move (ΔL=0, ΔR=%d) — "
                "check left encoder wiring or motor driver channel",
                label, dR,
            )
        if dR == 0:
            logger.warning(
                "%s: RIGHT encoder did not move (ΔL=%d, ΔR=0) — "
                "check right encoder wiring or motor driver channel",
                label, dL,
            )

        # ── Sync error ───────────────────────────────────────────────────
        if dL > 0 and dR > 0:
            sync_err   = abs(dL - dR)
            max_travel = max(dL, dR)
            ratio      = sync_err / max_travel
            if ratio > config.ENCODER_SYNC_WARN_RATIO:
                logger.warning(
                    "%s: high left-right sync error — "
                    "ΔL=%d ΔR=%d diff=%d (%.1f%%) — robot likely drifted; "
                    "consider tuning SYNC_KP/KI in firmware",
                    label, dL, dR, sync_err, ratio * 100,
                )
            else:
                logger.debug(
                    "%s: sync OK — ΔL=%d ΔR=%d diff=%d (%.1f%%)",
                    label, dL, dR, sync_err, ratio * 100,
                )

        # ── Target coverage ──────────────────────────────────────────────
        avg_travel = (dL + dR) / 2.0
        if expected_ticks > 0:
            coverage = avg_travel / expected_ticks
            if coverage < 0.80:
                logger.warning(
                    "%s: only %.0f%% of target ticks reached "
                    "(target=%d ticks, actual avg=%.0f) — "
                    "possible stall, slip, or TICKS_PER_CM needs calibration",
                    label, coverage * 100, expected_ticks, avg_travel,
                )
            else:
                logger.debug(
                    "%s: coverage %.0f%% (target=%d ticks, actual avg=%.0f)",
                    label, coverage * 100, expected_ticks, avg_travel,
                )

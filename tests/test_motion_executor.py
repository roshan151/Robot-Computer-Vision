"""
MotionExecutor / Gesturer semantics — no hardware required.

Run:  python -m pytest tests/test_motion_executor.py -v
      python tests/test_motion_executor.py          (plain runner, no pytest)

FakeMovement emulates the one property that makes the real bridge awkward: a
move is a blocking call that only ends early if someone brakes it out of band.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import config
from gestures import NO, YES, Gesturer
from motion_executor import CANCELLED, DONE, FAILED, MotionExecutor

MOVE_SECONDS = 0.40


class FakeMovement:
    """Stand-in for ArduinoMovement with realistic blocking + interrupt."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, float]] = []
        self.estops = 0
        self._brake = threading.Event()
        self._lock = threading.Lock()

    def _blocking_move(self, name: str, value: float) -> None:
        with self._lock:
            self.calls.append((name, value))
        self._brake.clear()
        # Interruptible sleep: the real move() returns early (by raising) when
        # the firmware reports a STOP-terminated move.
        if self._brake.wait(timeout=MOVE_SECONDS):
            raise RuntimeError("Arduino returned ERR — move stopped [STOP]")

    def straight(self, meters=None): self._blocking_move("straight", meters)
    def reverse(self, meters=None):  self._blocking_move("reverse", meters)
    def left(self, angle=None):      self._blocking_move("left", angle)
    def right(self, angle=None):     self._blocking_move("right", angle)
    def stop(self, _unused=None):    pass

    def emergency_stop(self) -> None:
        self.estops += 1
        self._brake.set()



def _drain_events(ex: MotionExecutor) -> list:
    out = []
    while not ex.events.empty():
        out.append(ex.events.get_nowait())
    return out


# --------------------------------------------------------------------------- #

def test_submit_is_non_blocking() -> None:
    move = FakeMovement()
    with MotionExecutor(move) as ex:
        t0 = time.monotonic()
        ex.submit("straight", 3.0)
        assert time.monotonic() - t0 < 0.05, "submit() blocked"
        ex.cancel_all("teardown")


def test_cancel_interrupts_move_in_flight() -> None:
    """The whole point: a move already running must be stoppable."""
    move = FakeMovement()
    with MotionExecutor(move) as ex:
        ex.submit("straight", 3.0)
        time.sleep(0.10)                       # let the worker start it
        t0 = time.monotonic()
        ex.cancel_all("stop command")
        assert ex.drain(timeout=2.0)
        halt = time.monotonic() - t0

        assert move.estops >= 1, "emergency_stop was never called"
        assert halt < 0.30, f"halt took {halt:.3f}s"
        ev = _drain_events(ex)
        assert [e.status for e in ev] == [CANCELLED], ev


def test_cancel_drops_queued_jobs() -> None:
    move = FakeMovement()
    with MotionExecutor(move) as ex:
        for _ in range(4):
            ex.submit("straight", 1.0)
        time.sleep(0.05)
        ex.cancel_all("stop")
        assert ex.drain(timeout=2.0)
        ev = _drain_events(ex)
        assert len(ev) == 4
        assert all(e.status == CANCELLED for e in ev), ev


def test_normal_move_completes() -> None:
    move = FakeMovement()
    with MotionExecutor(move) as ex:
        ex.submit("left", 30)
        assert ex.drain(timeout=2.0)
        ev = _drain_events(ex)
        assert [e.status for e in ev] == [DONE], ev
        assert move.calls == [("left", 30)]


def test_gesture_reaches_the_drivetrain() -> None:
    move = FakeMovement()
    with MotionExecutor(move) as ex:
        Gesturer(ex).yes()
        assert ex.drain(timeout=3.0)
        assert [c[0] for c in move.calls] == ["straight", "reverse"], move.calls


def test_gesture_shapes() -> None:
    assert YES == [("straight", config.GESTURE_YES_METERS),
                   ("reverse", config.GESTURE_YES_METERS)]
    assert NO == [("left", 30.0), ("right", 60.0), ("left", 30.0)]
    net = sum(v if op == "right" else -v for op, v in NO)
    assert net == 0, "NO gesture is not net-zero"


def test_gesture_dropped_not_queued() -> None:
    move = FakeMovement()
    with MotionExecutor(move) as ex:
        g = Gesturer(ex)
        accepted = [g.yes() for _ in range(5)]
        assert sum(a is not None for a in accepted) == 1, \
            "rapid gestures were queued instead of dropped"
        assert ex.drain(timeout=3.0)


def test_gesture_yields_to_real_motion() -> None:
    move = FakeMovement()
    with MotionExecutor(move) as ex:
        g = Gesturer(ex)
        ex.submit("straight", 3.0)
        time.sleep(0.10)
        assert g.no() is None, "gesture ran while the robot was driving"
        ex.cancel_all("teardown")
        assert ex.drain(timeout=2.0)


def test_failure_is_not_reported_as_cancelled() -> None:
    move = FakeMovement()

    def boom(_v=None):
        raise RuntimeError("serial exploded")

    move.right = boom                                    # type: ignore[assignment]
    with MotionExecutor(move) as ex:
        ex.submit("right", 90)
        assert ex.drain(timeout=2.0)
        ev = _drain_events(ex)
        assert [e.status for e in ev] == [FAILED], ev
        assert "serial exploded" in ev[0].detail


def test_status_reports_motion() -> None:
    move = FakeMovement()
    with MotionExecutor(move) as ex:
        assert ex.status()["moving"] is False
        ex.submit("straight", 3.0)
        time.sleep(0.10)
        st = ex.status()
        assert st["moving"] is True
        assert st["gesturing"] is False
        assert st["current"]["op"] == "straight"
        ex.cancel_all("teardown")


def test_default_speed_is_70_percent() -> None:
    assert config.DEFAULT_SPEED_PERCENT == 70.0
    from drivetrain.drivetrain_client import duty_from_percent
    # 70 % of 255 = 178.5; Python's round() breaks the tie to even -> 178.
    assert duty_from_percent(config.DEFAULT_SPEED_PERCENT) == 178


def test_estop_seq_band_is_disjoint() -> None:
    """Firmware dedupes on the previous seq, so the e-stop must never reuse
    the in-flight move's sequence number."""
    assert config.NORMAL_SEQ_MAX < config.ESTOP_SEQ_MIN <= config.ESTOP_SEQ_MAX <= 255


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"  FAIL  {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)

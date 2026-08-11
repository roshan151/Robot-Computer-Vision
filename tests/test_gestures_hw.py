"""
Hardware smoke test for the gesture channel and the emergency-stop path.

REQUIRES A REAL ROBOT ON THE FLOOR, WITH ROOM TO MOVE.
The cancel test drives forward up to 3 m before braking.

  python tests/test_gestures_hw.py --check estop
  python tests/test_gestures_hw.py --check latency
  python tests/test_gestures_hw.py --check gestures
  python tests/test_gestures_hw.py                 # all of the above

What each check is for
----------------------
estop     The one that matters. ArduinoBridge.move() holds _cmd_lock for the
          whole move, so a plain stop() from another thread cannot interrupt
          it. This proves the out-of-band path does. If this fails, nothing
          downstream is safe.

latency   Times right(30) at the configured default PWM and reports the NO
          gesture budget (three turns). Under ~1.2 s the gesture channel works
          as the robot's primary reply; over ~2.5 s it does not and the phone
          UI stops being optional.

gestures  Plays YES and NO and confirms neither leaked into MovementHistory.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import config
from gestures import Gesturer
from motion_executor import CANCELLED, DONE, MotionExecutor
from movement_adapter import ArduinoMovement, MovementHistory
from movement_context import MovementContext

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
log = logging.getLogger("gesture-hw")

HALT_BUDGET_S = 0.5


def check_estop(ex: MotionExecutor, move: ArduinoMovement) -> bool:
    print("\n=== emergency stop during a move ===")
    print("    driving 3 m, braking after 1 s — stand clear")
    ex.submit("straight", 3.0)
    time.sleep(1.0)

    t0 = time.monotonic()
    ex.cancel_all("hardware test")
    ex.drain(timeout=config.MOVE_TIMEOUT_S + 5)
    halt = time.monotonic() - t0

    ev = ex.events.get_nowait() if not ex.events.empty() else None
    print(f"    halt latency : {halt:.3f} s  (budget {HALT_BUDGET_S})")
    print(f"    event        : {ev.status if ev else 'NONE'}")

    counts = move.drivetrain.move_counts()
    expected = int(3.0 * config.TICKS_PER_CM * 100)
    actual = (abs(counts["left"]) + abs(counts["right"])) / 2
    print(f"    encoder ticks: {actual:.0f} of {expected} target "
          f"({actual / expected * 100:.0f}%)")

    ok = True
    if ev is None or ev.status != CANCELLED:
        print("    FAIL: move did not report as cancelled")
        ok = False
    if halt > HALT_BUDGET_S:
        print(f"    FAIL: halt took {halt:.3f}s — the S frame is queueing "
              f"behind _cmd_lock instead of going out of band")
        ok = False
    if actual >= expected * 0.95:
        print("    FAIL: the move ran to completion — it was never interrupted")
        ok = False
    print("    PASS" if ok else "    FAILED")
    return ok


def check_latency(ex: MotionExecutor) -> bool:
    print("\n=== gesture latency at "
          f"{config.DEFAULT_SPEED_PERCENT:.0f}% PWM ===")
    samples = []
    for i in range(3):
        t0 = time.monotonic()
        ex.submit("right", config.GESTURE_NO_DEGREES)
        ex.drain(timeout=15)
        samples.append(time.monotonic() - t0)
        print(f"    right({config.GESTURE_NO_DEGREES:.0f}) #{i + 1}: "
              f"{samples[-1]:.3f} s")
        time.sleep(0.3)
        ex.submit("left", config.GESTURE_NO_DEGREES)     # undo, keep it in place
        ex.drain(timeout=15)
        time.sleep(0.3)

    per_turn = sum(samples) / len(samples)
    no_total = per_turn * 3
    print(f"\n    mean per turn : {per_turn:.3f} s")
    print(f"    NO gesture    : {no_total:.3f} s  (3 turns)")
    if no_total < 1.2:
        print("    PASS — gesture channel is fast enough to be the primary reply")
        return True
    if no_total < 2.5:
        print("    MARGINAL — usable, but build the phone UI early")
        return True
    print("    FAIL — too slow to be the primary reply channel; "
          "Phase 4 (phone UI) is now mandatory")
    return False


def check_gestures(ex: MotionExecutor, hist: MovementHistory) -> bool:
    print("\n=== gesture vocabulary ===")
    g = Gesturer(ex)
    ok = True
    for name in ("yes", "no"):
        print(f"    playing {name.upper()} ...")
        t0 = time.monotonic()
        jobs = g.play(name)
        if jobs is None:
            print("    FAIL: gesture was suppressed with an idle robot")
            ok = False
            continue
        ex.drain(timeout=30)
        statuses = []
        while not ex.events.empty():
            statuses.append(ex.events.get_nowait().status)
        print(f"        {time.monotonic() - t0:.3f} s, steps: {statuses}")
        if any(s != DONE for s in statuses):
            print("    FAIL: a gesture step did not complete")
            ok = False

    if hist.stack:
        print(f"    FAIL: gestures leaked into MovementHistory: {hist.stack}")
        print("          origin() would replay the conversation backwards")
        ok = False
    else:
        print("    history stack clean — origin() is unaffected by gestures")

    print("    PASS" if ok else "    FAILED")
    return ok


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", choices=["estop", "latency", "gestures", "all"],
                    default="all")
    ap.add_argument("--port", default=config.SERIAL_PORT,
                    help="Serial port (default: config.SERIAL_PORT / ROBOT_SERIAL_PORT).")
    ap.add_argument("--baud", type=int, default=config.BAUD_RATE,
                    help="Baud rate (default: config.BAUD_RATE).")
    args = ap.parse_args()

    config.SERIAL_PORT = args.port
    config.BAUD_RATE = args.baud
    ctx = MovementContext()
    move = ArduinoMovement(ctx=ctx)
    hist = MovementHistory(move)
    results = {}

    with MotionExecutor(move, hist) as ex:
        try:
            if args.check in ("estop", "all"):
                results["estop"] = check_estop(ex, move)
            if args.check in ("latency", "all"):
                results["latency"] = check_latency(ex)
            if args.check in ("gestures", "all"):
                results["gestures"] = check_gestures(ex, hist)
        finally:
            ex.cancel_all("test teardown")
            move.close()

    print("\n" + "=" * 46)
    for name, ok in results.items():
        print(f"  {name:10s} {'PASS' if ok else 'FAIL'}")
    sys.exit(0 if all(results.values()) else 1)


if __name__ == "__main__":
    main()

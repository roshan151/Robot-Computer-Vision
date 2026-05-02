"""
Slow brain loop (~5–15 Hz): vision / decisions → intent commands on Arduino.

Run from repository root:
  python brain_loop.py
"""

from __future__ import annotations

import logging
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from drivetrain_client import SerialDrivetrain

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

try:
    import cv2  # type: ignore
except ImportError:
    cv2 = None


def decide_action(frame_ok: bool, stub_obstacle: bool) -> str:
    """Return one of F, B, L, R, S — placeholder policy."""
    if stub_obstacle:
        return "L"
    if frame_ok:
        return "F"
    return "S"


def main() -> None:
    dt = SerialDrivetrain()
    dt.set_base_speed_percent(55.0)

    cap = None
    if cv2 is not None:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            logger.warning("no camera; using stub frames")
            cap = None

    period = 1.0 / 10.0
    stub_toggle = False

    try:
        while True:
            t0 = time.monotonic()
            frame_ok = False
            if cap is not None:
                ok, frame = cap.read()
                frame_ok = ok and frame is not None
                # Future: run model / lane detector on `frame`
            else:
                frame_ok = True

            stub_toggle = not stub_toggle
            action = decide_action(frame_ok, stub_obstacle=stub_toggle and False)

            if action == "F":
                dt.intent_forward()
            elif action == "B":
                dt.intent_backward()
            elif action == "L":
                dt.intent_left()
            elif action == "R":
                dt.intent_right()
            else:
                dt.stop()

            enc = dt.get_encoder_status()
            logger.debug("enc %s action %s", enc, action)

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, period - elapsed))
    except KeyboardInterrupt:
        logger.info("stop requested")
    finally:
        dt.stop()
        dt.close()
        if cap is not None:
            cap.release()


if __name__ == "__main__":
    main()

"""
Integrated robot entry: voice + GPT + vision service + Arduino drivetrain.

From repository root:
  python run_robot.py
  python run_robot.py --no-guardian
  python run_robot.py --brain-only

Vision API (separate terminal, optional):
  cd deprecated && uvicorn Vision.app:app --host 0.0.0.0 --port 8080
(Or see README for the exact path on your machine.)
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parent
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

logging.basicConfig(level=logging.INFO)


def main() -> None:
    parser = argparse.ArgumentParser(description="Voice + vision + Arduino stack")
    parser.add_argument(
        "--no-guardian",
        action="store_true",
        help="Disable background vision stop (VISION_HALT_OBJECTS)",
    )
    parser.add_argument(
        "--brain-only",
        action="store_true",
        help="Run OpenCV intent loop only (no voice / GPT)",
    )
    args = parser.parse_args()

    if args.brain_only:
        from brain_loop import main as brain_main

        brain_main()
        return

    from voice_session import run_voice_session

    run_voice_session(start_guardian=not args.no_guardian)


if __name__ == "__main__":
    main()

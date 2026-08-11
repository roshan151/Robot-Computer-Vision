"""
Integrated robot entry: voice + GPT + vision service + Arduino drivetrain.

From repository root:
  python run_robot.py                 # voice + GPT + vision guardian
  python run_robot.py --voice-only    # voice + GPT, no camera required
  python run_robot.py --no-guardian   # same as --voice-only (kept as alias)

Vision API (separate terminal, optional, only for the default mode):
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
        "--voice-only",
        action="store_true",
        help="Voice + GPT control with the vision guardian disabled — "
             "for running without a camera connected.",
    )
    parser.add_argument(
        "--no-guardian",
        action="store_true",
        help="Alias for --voice-only (kept for compatibility).",
    )
    args = parser.parse_args()

    from voice_session import run_voice_session

    run_voice_session(start_guardian=not (args.voice_only or args.no_guardian))


if __name__ == "__main__":
    main()

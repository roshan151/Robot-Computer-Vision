"""
Legacy filename preserved for convenience.

Integrated stack (voice → GPT → Arduino + optional vision guardian):
  python -m robust.raspberry_pi.run_robot

Start the vision API (YOLO) in another process:
  uvicorn Vision.app:app --host 0.0.0.0 --port 8080
"""

from __future__ import annotations

import logging
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

logging.basicConfig(level=logging.INFO)


def main() -> None:
    from robust.raspberry_pi.voice_session import run_voice_session

    run_voice_session()


if __name__ == "__main__":
    main()

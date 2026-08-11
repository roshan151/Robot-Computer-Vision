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
import socket
from pathlib import Path

_REPO = Path(__file__).resolve().parent
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

import robot_log

# Registered by main() once the drivetrain exists, so the fatal path can brake.
_MOVE = None


def _emergency_brake(cause: str) -> None:
    """Last action before the process dies: get the motors off.

    Runs from sys.excepthook / threading.excepthook / the signal handler, so it
    must never raise — a failure here would mask the fault we are trying to
    report. The firmware's link watchdog is the backstop for SIGKILL, which no
    handler can intercept.
    """
    if _MOVE is None:
        return
    try:
        _MOVE.emergency_stop()
        robot_log.event("estop", logging.CRITICAL, reason=f"process dying: {cause}")
    except Exception as e:
        robot_log.event("estop", logging.CRITICAL, ok=False,
                        reason=f"process dying: {cause}",
                        err=f"{type(e).__name__}: {e}")

sockets = [
    "/tmp/pisugar-server.sock",
    "/tmp/pisugar.sock"
]
def pisugar_command(command):


    for path in sockets:
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
                s.connect(path)
                s.sendall((command + "\n").encode())
                return s.recv(1024).decode().strip()
        except (FileNotFoundError, ConnectionRefusedError):
            pass

    raise RuntimeError("PiSugar Power Manager not running")

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
    parser.add_argument(
        "--log", default=None,
        help="Path to the JSON Lines event log (default: config.LOG_PATH).",
    )
    args = parser.parse_args()

    log_path = robot_log.setup(args.log)
    robot_log.install_crash_handlers(on_fatal=_emergency_brake)
    robot_log.event(
        "session.start",
        mode="voice-only" if (args.voice_only or args.no_guardian) else "full",
        pid=__import__("os").getpid(),
        port=__import__("config").SERIAL_PORT,
        speed_pct=__import__("config").DEFAULT_SPEED_PERCENT,
        log=str(log_path),
    )

    from voice_session import run_voice_session

    try:
        run_voice_session(start_guardian=not (args.voice_only or args.no_guardian))
    except Exception:
        # Re-raised so sys.excepthook logs the cause and brakes; this only
        # exists to make the ordering explicit.
        raise
    else:
        robot_log.event("session.stop", reason="clean exit")

if __name__ == "__main__":
    main()

"""
Spoken output — used sparingly and deliberately.

The robot is silent during conversation: it answers with gestures, and its own
voice in the microphone is the failure mode the whole design avoids.  Speech is
reserved for two moments where nothing is listening anyway:

  * startup, before the microphone is ever opened (battery report)
  * failure, after the voice session has been torn down

Both are safe for the same reason: no capture stream is live when they play.

Backends, in order:
  1. Nix TTS       — good quality, needs NIX_TTS_DIR/NIX_TTS_MODEL and numpy
  2. espeak-ng     — robotic, but `apt install espeak-ng` and it always works
  3. log only      — the text still reaches logs.json

espeak-ng matters more than it looks. Nix TTS needs a model directory that may
not be present, imports torch-adjacent machinery, and takes seconds to load —
none of which you want on the failure path, where speech is most needed and the
system is least healthy.
"""

from __future__ import annotations

import logging
import shutil
import subprocess
import sys
import threading
from typing import Optional

import config
import robot_log

logger = logging.getLogger(__name__)

_lock = threading.Lock()
_nix = None
_sd = None
_backend: Optional[str] = None


def _init() -> str:
    """Pick a backend once. Never raises."""
    global _nix, _sd, _backend
    if _backend is not None:
        return _backend

    if config.NIX_TTS_DIR:
        try:
            import sounddevice as sd  # type: ignore

            if config.NIX_TTS_DIR not in sys.path:
                sys.path.insert(0, config.NIX_TTS_DIR)
            from nix.models.TTS import NixTTSInference  # type: ignore

            _nix = NixTTSInference(
                model_dir=config.NIX_TTS_MODEL or config.NIX_TTS_DIR
            )
            _sd = sd
            _backend = "nix"
            return _backend
        except Exception as e:
            logger.debug("Nix TTS unavailable: %s", e)

    for exe in ("espeak-ng", "espeak"):
        if shutil.which(exe):
            _backend = exe
            return _backend

    _backend = "none"
    logger.warning("no TTS backend — spoken output will only reach logs.json")
    return _backend


def available() -> bool:
    return _init() != "none"


def backend() -> str:
    return _init()


def say(text: str, *, event: str = "voice.say", **fields) -> bool:
    """Speak `text` and block until finished. Always logged; never raises.

    Blocking matters: callers speak immediately before opening the microphone
    or immediately before exiting, and returning early would let the speech
    overlap with capture or be cut off by process exit.
    """
    robot_log.event(event, text=text, **fields)

    b = _init()
    if b == "none":
        return False

    with _lock:                       # two overlapping utterances help nobody
        try:
            if b == "nix":
                c, c_len, _ = _nix.tokenize(text)
                xw = _nix.vocalize(c, c_len)
                _sd.play(xw[0, 0], 22050, blocking=True)
                return True
            subprocess.run(
                [b, "-s", str(config.SPEECH_WPM), "-a", str(config.SPEECH_AMPLITUDE), text],
                timeout=30, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            return True
        except Exception as e:
            robot_log.event("audio.error", logging.WARNING,
                            stage="tts", backend=b, err=f"{type(e).__name__}: {e}")
            return False


if __name__ == "__main__":
    text = " ".join(sys.argv[1:]) or "Battery ninety two percent, four point one volts."
    robot_log.setup()
    print(f"backend: {backend()}")
    print("spoke:", say(text))

"""
Short non-speech tones — the robot's only sound during normal operation.

Why tones and not speech
------------------------
A spoken prompt is long, lands in the middle of the speech band, and is what
the microphone is most likely to mistake for a command.  A 120 ms tone is over
before the microphone stream is even opened, and carries the one bit that
actually matters when running headless: "I am listening now."

Ordering is the whole trick.  The cue is played BEFORE the capture stream is
opened, never while it is live.  If the stream is already open the tone lands in
the buffer, and `Recognizer.listen()` reads that buffer — it would treat the
robot's own beep as the start of your utterance and either prefix your command
with a chirp or clip the phrase early.  `guard_s` then covers the room's
reverb tail before capture begins.

Playback backends, in order:
    1. sounddevice + numpy      (in-process, exact timing)
    2. aplay / paplay           (subprocess, no numpy needed)
    3. silence                  (log it and carry on)

Nothing here raises. An audio cue failing must never take the robot down, and
on the failure path this module is expected to run when other things are
already broken.
"""

from __future__ import annotations

import io
import logging
import math
import shutil
import struct
import subprocess
import time
import wave
from typing import Dict, Optional, Tuple

import config
import robot_log

logger = logging.getLogger(__name__)

SAMPLE_RATE = 16000

# (frequency Hz, duration ms, gain 0-1)
# EXACTLY TWO TONES. The Live session holds the microphone open continuously,
# so every sound the robot makes is heard by the model as if you had said it.
# A per-turn "ready" chirp is therefore not a convenience, it is interference —
# and there is no listening state to announce any more, because the robot is
# always listening.
#
# What survives are the two moments where a tone carries information no other
# channel can, and where nothing is being interrupted:
#
#   started  the session is up and connected. Plays BEFORE the stream opens.
#   error    the session has died. Plays AFTER it is torn down.
#
# Do not add a third without a reason that beats "it goes into the microphone".
TONES: Dict[str, Tuple[Tuple[int, int], ...]] = {
    # Rising two-note chirp: awake, connected, listening.
    "started": ((880, 70), (1320, 90)),
    # Harsh low triple: something is wrong, check logs.json.
    "error": ((330, 140), (330, 140), (247, 220)),
}


def _render(spec: Tuple[Tuple[int, int], ...], gain: float) -> bytes:
    """16-bit mono PCM with 5 ms raised-cosine edges.

    The fades are not cosmetic: a square-edged tone clicks, and a click is
    broadband energy that a voice activity detector is far more likely to
    latch onto than the tone itself.
    """
    out = bytearray()
    for freq, ms in spec:
        n = int(SAMPLE_RATE * ms / 1000)
        edge = max(1, int(SAMPLE_RATE * 0.005))
        for i in range(n):
            env = 1.0
            if i < edge:
                env = 0.5 - 0.5 * math.cos(math.pi * i / edge)
            elif i > n - edge:
                env = 0.5 - 0.5 * math.cos(math.pi * (n - i) / edge)
            s = math.sin(2 * math.pi * freq * i / SAMPLE_RATE) * env * gain
            out += struct.pack("<h", int(max(-1.0, min(1.0, s)) * 32767))
    return bytes(out)


def _to_wav(pcm: bytes) -> bytes:
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SAMPLE_RATE)
        w.writeframes(pcm)
    return buf.getvalue()


class AudioCues:
    """Renders every tone once at construction; playback is then just I/O."""

    def __init__(
        self,
        enabled: Optional[bool] = None,
        device: Optional[str] = None,
        gain: Optional[float] = None,
        guard_s: Optional[float] = None,
    ) -> None:
        self.enabled = config.AUDIO_CUES_ENABLED if enabled is None else enabled
        self.device = device if device is not None else config.AUDIO_CUE_DEVICE
        self.guard_s = config.AUDIO_CUE_GUARD_S if guard_s is None else guard_s
        g = config.AUDIO_CUE_GAIN if gain is None else gain

        self._pcm = {name: _render(spec, g) for name, spec in TONES.items()}
        self._wav = {name: _to_wav(p) for name, p in self._pcm.items()}
        self._sd = None
        self._np = None
        self._player = self._pick_backend()

    def _pick_backend(self) -> str:
        try:
            import numpy as np  # type: ignore
            import sounddevice as sd  # type: ignore

            self._sd, self._np = sd, np
            return "sounddevice"
        except Exception as e:
            logger.debug("sounddevice/numpy unavailable for cues: %s", e)
        for exe in ("paplay", "aplay"):
            if shutil.which(exe):
                return exe
        # Structured, not just a log line: with no screen and no voice, "the
        # robot is mute" has to be greppable in logs.json rather than buried
        # in console output nobody is watching.
        robot_log.event(
            "audio.error", logging.WARNING, stage="cue-backend",
            err="no playback backend (no sounddevice/numpy, no aplay, no paplay)",
            fix="pip install sounddevice numpy, or apt install alsa-utils",
        )
        return "none"

    # ------------------------------------------------------------------ #

    def play(self, name: str, guard: bool = True) -> bool:
        """Play a cue and block until it has finished.

        Blocking is deliberate. The caller opens the microphone immediately
        afterwards, and returning early would put the tone into the capture
        buffer — exactly the interference this module exists to avoid.

        `guard` adds a short settle before returning, for the room's reverb
        tail. Skip it only when nothing is about to listen.
        """
        if not self.enabled or self._player == "none":
            return False
        pcm = self._pcm.get(name)
        if pcm is None:
            logger.warning("unknown audio cue %r", name)
            return False

        ok = False
        try:
            if self._player == "sounddevice":
                data = self._np.frombuffer(pcm, dtype="<i2")
                kw = {"device": self.device} if self.device else {}
                self._sd.play(data, SAMPLE_RATE, blocking=True, **kw)
                ok = True
            else:
                cmd = [self._player]
                if self._player == "aplay":
                    cmd += ["-q"]
                    if self.device:
                        cmd += ["-D", self.device]
                elif self.device:
                    cmd += ["--device", self.device]
                cmd += ["-"]
                subprocess.run(cmd, input=self._wav[name], timeout=5,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                ok = True
        except Exception as e:
            # Never propagate. A missing speaker must not stop the robot, and
            # on the failure path this runs when things are already broken.
            logger.warning("audio cue %r failed (%s): %s", name, self._player, e)

        if ok and guard and self.guard_s > 0:
            time.sleep(self.guard_s)
        return ok

    def started(self) -> bool:
        """Session is up. Play this BEFORE the microphone stream opens, so the
        tone cannot land in the audio being streamed to the model."""
        return self.play("started")

    def error(self) -> bool:
        """Session has died. Play this AFTER it is torn down, for the same
        reason in reverse: by then nothing is listening."""
        return self.play("error", guard=False)

    def save_wavs(self, directory: str) -> list:
        """Write the cues out as .wav — for auditioning them off-robot."""
        from pathlib import Path

        d = Path(directory)
        d.mkdir(parents=True, exist_ok=True)
        written = []
        for name, data in self._wav.items():
            p = d / f"cue_{name}.wav"
            p.write_bytes(data)
            written.append(str(p))
        return written


_default: Optional[AudioCues] = None


def cues() -> AudioCues:
    """Process-wide instance; tones are rendered once."""
    global _default
    if _default is None:
        _default = AudioCues()
    return _default


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Audition the audio cues.")
    ap.add_argument("cue", nargs="?", default="all", choices=["all", *TONES])
    ap.add_argument("--save", metavar="DIR", help="write .wav files instead of playing")
    ap.add_argument("--device", default=config.AUDIO_CUE_DEVICE or None)
    args = ap.parse_args()

    c = AudioCues(enabled=True, device=args.device)
    if args.save:
        for p in c.save_wavs(args.save):
            print(p)
    else:
        print(f"backend: {c._player}")
        for name in (TONES if args.cue == "all" else [args.cue]):
            print(f"  {name}")
            c.play(name, guard=False)
            time.sleep(0.5)

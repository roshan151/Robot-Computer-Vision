"""
Local speech-likeness gate — decides whether audio is worth uploading.

`Recognizer.listen()` is an energy detector, not a speech detector. A door
click, a motor whine, a chair scrape, or a cough all cross the threshold and
each one becomes a Gemini request. With `dynamic_energy_threshold` on, a quiet
room makes this worse over time: the threshold drifts down until the robot is
uploading its own fan.

So before spending a request, check cheaply and locally whether the clip could
plausibly be speech. Three properties separate a spoken command from the things
that actually trigger the microphone:

  duration        a command is a few hundred ms at minimum; a click is ~50 ms
  voiced time     speech sustains energy across many frames; a transient is one
                  loud frame surrounded by silence
  modulation      speech has syllable-rate amplitude variation. A fan or a
                  motor is loud and sustained but *flat*, which is exactly the
                  case a plain energy threshold cannot reject

All arithmetic is on 20 ms frames of 16-bit PCM using the stdlib only — no
numpy, no audioop (removed in 3.13). It costs well under a millisecond on a Pi,
against ~1 s and one quota unit for the request it prevents.
"""

from __future__ import annotations

import io
import logging
import wave
from array import array
from dataclasses import dataclass
from typing import Optional

import config

logger = logging.getLogger(__name__)

FRAME_MS = 20
FULL_SCALE = 32768.0


@dataclass
class GateResult:
    accepted: bool
    reason: str = ""
    duration_s: float = 0.0
    peak: float = 0.0            # 0-1, loudest frame
    voiced_s: float = 0.0        # seconds of frames near the peak
    modulation: float = 0.0      # peak / median frame energy

    def as_dict(self) -> dict:
        return {
            "dur": round(self.duration_s, 2),
            "peak": round(self.peak, 4),
            "voiced_s": round(self.voiced_s, 2),
            "mod": round(self.modulation, 1),
        }


def _frame_rms(samples: array, start: int, n: int) -> float:
    total = 0
    for i in range(start, min(start + n, len(samples))):
        s = samples[i]
        total += s * s
    count = min(n, len(samples) - start)
    if count <= 0:
        return 0.0
    return (total / count) ** 0.5 / FULL_SCALE


def measure(wav_bytes: bytes) -> GateResult:
    """Compute the gate metrics for a WAV clip. Never raises."""
    try:
        with wave.open(io.BytesIO(wav_bytes), "rb") as w:
            rate = w.getframerate()
            width = w.getsampwidth()
            channels = w.getnchannels()
            frames = w.readframes(w.getnframes())
    except Exception as e:
        return GateResult(False, f"unreadable wav: {type(e).__name__}")

    if width != 2:
        # Only 16-bit is produced by our capture path; anything else means the
        # recorder changed and the thresholds below would be meaningless.
        return GateResult(True, "not 16-bit, gate skipped")

    samples = array("h")
    samples.frombytes(frames[: len(frames) - (len(frames) % 2)])
    if channels > 1:
        samples = array("h", samples[::channels])
    if not samples:
        return GateResult(False, "empty")

    duration = len(samples) / rate
    n = max(1, int(rate * FRAME_MS / 1000))
    rms = [_frame_rms(samples, i, n) for i in range(0, len(samples), n)]
    if not rms:
        return GateResult(False, "empty")

    peak = max(rms)
    ordered = sorted(rms)
    median = ordered[len(ordered) // 2] or 1e-9
    # Frames within 12 dB of the peak count as carrying the utterance.
    voiced = sum(1 for r in rms if r > max(peak * 0.25, config.GATE_MIN_PEAK * 0.5))
    voiced_s = voiced * FRAME_MS / 1000.0

    return GateResult(True, "", duration, peak, voiced_s, peak / median)


def check(wav_bytes: bytes) -> GateResult:
    """Accept or reject a clip before it costs a request.

    Deliberately lenient: a rejected command is worse than an extra upload, so
    every threshold sits well below normal speech. The goal is to catch clicks,
    bumps and steady hum — not to be a real VAD.
    """
    if not config.GATE_ENABLED:
        return GateResult(True, "gate disabled")

    r = measure(wav_bytes)
    if not r.accepted:
        return r

    if r.duration_s < config.GATE_MIN_DURATION_S:
        return GateResult(False, "too short", r.duration_s, r.peak,
                          r.voiced_s, r.modulation)

    if r.peak < config.GATE_MIN_PEAK:
        return GateResult(False, "too quiet", r.duration_s, r.peak,
                          r.voiced_s, r.modulation)

    if r.voiced_s < config.GATE_MIN_VOICED_S:
        # One loud frame in a sea of silence: a click, a bump, a door.
        return GateResult(False, "transient, not speech", r.duration_s, r.peak,
                          r.voiced_s, r.modulation)

    if r.modulation < config.GATE_MIN_MODULATION:
        # Loud and sustained but flat — a fan, a motor, mains hum. This is the
        # case a plain energy threshold always lets through.
        return GateResult(False, "steady noise, no speech modulation",
                          r.duration_s, r.peak, r.voiced_s, r.modulation)

    return GateResult(True, "", r.duration_s, r.peak, r.voiced_s, r.modulation)


if __name__ == "__main__":
    import argparse
    import sys

    ap = argparse.ArgumentParser(
        description="Measure WAV clips against the speech gate. "
                    "Record a few real commands and a few of whatever is "
                    "triggering the robot, then tune the GATE_* values."
    )
    ap.add_argument("wavs", nargs="+")
    args = ap.parse_args()

    print(f"{'file':<34} {'dur':>6} {'peak':>7} {'voiced':>7} {'mod':>6}  verdict")
    for path in args.wavs:
        try:
            data = open(path, "rb").read()
        except OSError as e:
            print(f"{path:<34} {e}")
            continue
        r = check(data)
        verdict = "ACCEPT" if r.accepted else f"reject: {r.reason}"
        print(f"{path[-34:]:<34} {r.duration_s:6.2f} {r.peak:7.4f} "
              f"{r.voiced_s:7.2f} {r.modulation:6.1f}  {verdict}")

    print(f"\nthresholds: min_dur={config.GATE_MIN_DURATION_S}s "
          f"min_peak={config.GATE_MIN_PEAK} "
          f"min_voiced={config.GATE_MIN_VOICED_S}s "
          f"min_mod={config.GATE_MIN_MODULATION}")

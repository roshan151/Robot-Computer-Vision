"""
Spoken output — Gemini text-to-speech, cached to disk.

This module replaces the two things it was split across before: `audio_cues.py`
(rendered sine tones) and `speech.py` (espeak-ng / Nix TTS). Both are gone. The
robot now says what it means in one voice, and the ordering rules that made the
tones safe are unchanged and still the whole trick:

    * an utterance plays BEFORE the microphone stream is opened, never while
      it is live. If the stream is already open the audio lands in the capture
      buffer and is streamed to the model as if the operator had said it.
    * on the failure path it plays AFTER the session is torn down, for the same
      reason in reverse: by then nothing is listening.

`guard_s` covers the room's reverb tail after playback, before capture begins.

One key, one host
-----------------
Speech goes to generativelanguage.googleapis.com with the same GEMINI_API_KEY
the Live session uses. Google Cloud Text-to-Speech (WaveNet) would have been
cheaper per utterance, but it is a different API on a different host: AI Studio
keys are restricted to the Generative Language API, so it would have meant a
second credential on a billing-enabled Cloud project. At this volume — five
cached phrases and one battery line a boot — that trade is not worth a key.

The bill that does apply is per project, not per key, and the Live session is
on the same project. Which is one more reason the cache matters.

Why the cache is not optional
-----------------------------
Synthesis is a network call. The two moments the robot speaks are the two
moments the network is least trustworthy: boot (Wi-Fi may not be associated
yet) and failure (the session just died, possibly because the link did).
Anything that must be sayable when the network is down has to already be on
disk.

So: every synthesis is written to `TTS_CACHE_DIR` keyed by a hash of the text
and the voice parameters, and a cache hit never touches the network. Static
phrases are primed once — `prime()` at first successful boot is enough to make
the robot permanently able to announce its own failure offline.

Dynamic text (battery readings, error descriptions) misses the cache the first
time it is said and is synthesized live. If that call fails, `say()` falls back
to `fallback_text` when the caller supplied one — which is how the error path
still speaks something useful with no network at all.

The cache does a second job here that it would not have done for WaveNet. Gemini
TTS is a language model, so the same input renders differently on each call —
different pacing, different emphasis. Caching freezes one take, and the robot
says "Robot online" the same way every boot instead of reinterpreting it.

Playback backends, in order:
    1. sounddevice + numpy      (in-process, exact timing)
    2. aplay / paplay           (subprocess, no numpy needed)
    3. silence                  (logged, structured, and carry on)

Nothing here raises. Speech failing must never take the robot down, and on the
failure path this module runs when other things are already broken.
"""

from __future__ import annotations

import base64
import hashlib
import io
import json
import logging
import shutil
import subprocess
import threading
import time
import wave
from pathlib import Path
from typing import Dict, Optional

import config
import robot_log

logger = logging.getLogger(__name__)

GENERATE_URL = (
    "https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent"
)

# Statuses worth trying again. 500 is the documented random failure mode of the
# TTS models (they occasionally emit text tokens instead of audio and the server
# rejects its own response); 429/503 are load. A 400 or 403 is a configuration
# mistake and retrying it just makes the same mistake twice.
RETRY_STATUSES = frozenset({429, 500, 503})

# Phrases the robot must be able to say with no network. `prime()` renders
# these; after one successful boot they live in the cache forever.
#
# EXACTLY TWO EVENTS, for the same reason there were exactly two tones. The
# Live session holds the microphone open continuously, so every sound the robot
# makes is heard by the model as if you had said it. A per-turn announcement is
# not a convenience, it is interference — and there is no listening state to
# announce any more, because the robot is always listening.
#
#   connected  the session is up. Plays BEFORE the stream opens.
#   error      the session has died. Plays AFTER it is torn down.
#
# Do not add a third without a reason that beats "it goes into the microphone".
STATIC_PHRASES: Dict[str, str] = {
    "connected": "Robot online. Voice session connected.",
    # Said alone when there is no description, and used as the offline fallback
    # for the described form below.
    "error": "Voice session failed. Check the log.",
    "battery_unknown": "Battery level unknown.",
}

_lock = threading.Lock()
_sd = None
_np = None
_player: Optional[str] = None


# --------------------------------------------------------------------------- #
# Playback
# --------------------------------------------------------------------------- #

def _pick_backend() -> str:
    """Choose a playback backend once. Never raises."""
    global _sd, _np, _player
    if _player is not None:
        return _player

    try:
        import numpy as np  # type: ignore
        import sounddevice as sd  # type: ignore

        _sd, _np = sd, np
        _player = "sounddevice"
        return _player
    except Exception as e:
        logger.debug("sounddevice/numpy unavailable for playback: %s", e)

    for exe in ("paplay", "aplay"):
        if shutil.which(exe):
            _player = exe
            return _player

    # Structured, not just a log line: with no screen and no voice, "the robot
    # is mute" has to be greppable in logs.json rather than buried in console
    # output nobody is watching.
    robot_log.event(
        "audio.error", logging.WARNING, stage="playback-backend",
        err="no playback backend (no sounddevice/numpy, no aplay, no paplay)",
        fix="pip install sounddevice numpy, or apt install alsa-utils",
    )
    _player = "none"
    return _player


def backend() -> str:
    """Which playback backend is in use: sounddevice / aplay / paplay / none."""
    return _pick_backend()


def available() -> bool:
    """True if the robot can make sound at all — cache or network aside."""
    return config.TTS_ENABLED and _pick_backend() != "none"


def _wav_bytes(pcm_or_wav: bytes) -> bytes:
    """Gemini returns headerless 24 kHz 16-bit mono PCM, so we add the RIFF
    header ourselves. The check is for the day an endpoint returns a real WAV —
    wrapping one twice produces a file that plays four bytes of noise."""
    if pcm_or_wav[:4] == b"RIFF":
        return pcm_or_wav
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(config.TTS_SAMPLE_RATE)
        w.writeframes(pcm_or_wav)
    return buf.getvalue()


def _play_wav(data: bytes) -> bool:
    """Play a WAV and block until it has finished.

    Blocking is deliberate. Callers speak immediately before opening the
    microphone or immediately before exiting, and returning early would put the
    speech into the capture buffer or let process exit cut it off.
    """
    p = _pick_backend()
    if p == "none":
        return False

    try:
        if p == "sounddevice":
            with wave.open(io.BytesIO(data), "rb") as w:
                frames = w.readframes(w.getnframes())
                rate = w.getframerate()
                channels = w.getnchannels()
            arr = _np.frombuffer(frames, dtype="<i2")
            if channels > 1:
                arr = arr.reshape(-1, channels)
            kw = {"device": config.TTS_DEVICE} if config.TTS_DEVICE else {}
            _sd.play(arr, rate, blocking=True, **kw)
            return True

        cmd = [p]
        if p == "aplay":
            cmd += ["-q"]
            if config.TTS_DEVICE:
                cmd += ["-D", config.TTS_DEVICE]
        elif config.TTS_DEVICE:
            cmd += ["--device", config.TTS_DEVICE]
        cmd += ["-"]
        subprocess.run(cmd, input=data, timeout=30,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except Exception as e:
        # Never propagate. A missing speaker must not stop the robot.
        logger.warning("playback failed (%s): %s", p, e)
        return False


# --------------------------------------------------------------------------- #
# Cache + synthesis
# --------------------------------------------------------------------------- #

def _cache_dir() -> Path:
    d = Path(config.TTS_CACHE_DIR).expanduser()
    d.mkdir(parents=True, exist_ok=True)
    return d


def _cache_path(text: str) -> Path:
    """Key on everything that changes the audio, not just the words.

    Model, voice and style direction all alter the rendering, so a change to
    any of them has to miss the cache. Otherwise switching voices leaves the
    robot speaking in the old one until someone thinks to delete the directory.
    """
    key = "|".join((
        text,
        config.TTS_MODEL,
        config.TTS_VOICE,
        config.TTS_STYLE,
        str(config.TTS_SAMPLE_RATE),
    ))
    return _cache_dir() / f"{hashlib.sha1(key.encode()).hexdigest()}.wav"


def _prompt(text: str) -> str:
    """`Say clearly and calmly: Robot online.`

    The directive prefix is the shape Google's single-speaker example uses, and
    it matters for more than tone: a bare transcript can fail to trigger the
    speech classifier, which either rejects the request as PROHIBITED_CONTENT
    or — worse, because it is silent about it — makes the model read the text
    as if it were an instruction. Keep the style short for the same reason.
    """
    style = config.TTS_STYLE.strip().rstrip(":")
    return f"{style}: {text}" if style else text


def _extract_audio(payload: dict) -> Optional[bytes]:
    """Pull the first audio blob out of a generateContent response.

    Written as a walk rather than a fixed path because the audio has sat at
    two different depths across API revisions (`inlineData` under a part, and
    `output_audio` on the newer Interactions shape). A missing key here would
    present as a mute robot with a 200 in the log, which is the least
    debuggable failure this module has.
    """
    stack = [payload]
    while stack:
        node = stack.pop()
        if isinstance(node, dict):
            for k in ("inlineData", "inline_data", "output_audio", "outputAudio"):
                blob = node.get(k)
                if isinstance(blob, dict) and blob.get("data"):
                    return base64.b64decode(blob["data"])
            stack.extend(node.values())
        elif isinstance(node, list):
            stack.extend(node)
    return None


def _synthesize(text: str) -> Optional[bytes]:
    """One Gemini TTS request, with retries. WAV bytes, or None on failure."""
    try:
        api_key = config.require("GEMINI_API_KEY")
    except Exception as e:
        robot_log.event("audio.error", logging.WARNING, stage="tts-auth",
                        err=f"{type(e).__name__}: {e}",
                        fix="set GEMINI_API_KEY in /etc/robot.env")
        return None

    try:
        import requests  # imported here so config-only tools need no network stack
    except ImportError as e:
        robot_log.event("audio.error", logging.WARNING, stage="tts-import",
                        err=str(e), fix="pip install requests")
        return None

    url = GENERATE_URL.format(model=config.TTS_MODEL)
    body = {
        "contents": [{"parts": [{"text": _prompt(text)}]}],
        "generationConfig": {
            "responseModalities": ["AUDIO"],
            "speechConfig": {
                "voiceConfig": {
                    "prebuiltVoiceConfig": {"voiceName": config.TTS_VOICE},
                },
            },
        },
    }

    for attempt in range(1, config.TTS_RETRIES + 2):
        try:
            resp = requests.post(
                url,
                headers={"x-goog-api-key": api_key,
                         "Content-Type": "application/json"},
                json=body,
                timeout=config.TTS_TIMEOUT_S,
            )
            if resp.status_code == 200:
                audio = _extract_audio(resp.json())
                if audio:
                    return _wav_bytes(audio)
                # 200 with no audio is the documented "returned text tokens
                # instead" case leaking through. Retryable, and the body is
                # logged because it usually contains the model's excuse.
                robot_log.event("audio.error", logging.WARNING,
                                stage="tts-response", attempt=attempt,
                                err="200 with no audio in the response",
                                body=resp.text[:300])
            else:
                # Body quoted, not parsed: Google's message names the fix more
                # often than the status does ("model not found" for a non-TTS
                # model id, "API key not valid", quota exceeded, and so on).
                robot_log.event("audio.error", logging.WARNING, stage="tts-http",
                                status=resp.status_code, attempt=attempt,
                                model=config.TTS_MODEL, err=resp.text[:300])
                if resp.status_code not in RETRY_STATUSES:
                    return None                 # config error; retrying repeats it
        except Exception as e:
            robot_log.event("audio.error", logging.WARNING, stage="tts-synth",
                            attempt=attempt, err=f"{type(e).__name__}: {e}")

        if attempt <= config.TTS_RETRIES:
            time.sleep(min(2 ** (attempt - 1) * 0.5, 4.0))

    return None


def render(text: str, *, allow_network: bool = True) -> Optional[bytes]:
    """WAV bytes for `text` — from cache, else synthesized and cached.

    `allow_network=False` answers "can this be said offline right now?" without
    a request, which is what the failure path uses to pick its fallback.
    """
    path = _cache_path(text)
    try:
        if path.is_file() and path.stat().st_size > 0:
            return path.read_bytes()
    except OSError as e:
        logger.debug("tts cache read failed for %s: %s", path.name, e)

    if not allow_network:
        return None

    data = _synthesize(text)
    if data is None:
        return None

    try:
        # Write-then-rename: a truncated file from a power cut mid-write would
        # be served forever afterwards, and this robot loses power for a living.
        tmp = path.with_suffix(".part")
        tmp.write_bytes(data)
        tmp.replace(path)
    except OSError as e:
        logger.debug("tts cache write failed for %s: %s", path.name, e)
    return data


# --------------------------------------------------------------------------- #
# Speaking
# --------------------------------------------------------------------------- #

def say(
    text: str,
    *,
    event: str = "voice.say",
    guard: bool = True,
    fallback_text: Optional[str] = None,
    **fields,
) -> bool:
    """Speak `text` and block until finished. Always logged; never raises.

    `fallback_text` should be a phrase known to be in the cache. If `text`
    cannot be produced — no network, no key, API down — the fallback is spoken
    instead, so a novel sentence never degrades into silence at the exact
    moment the operator needs to hear something.

    `guard` adds a short settle before returning, for the room's reverb tail.
    Skip it only when nothing is about to listen.
    """
    robot_log.event(event, text=text, **fields)

    if not config.TTS_ENABLED or _pick_backend() == "none":
        return False

    with _lock:                       # two overlapping utterances help nobody
        data = render(text)
        spoken = text
        if data is None and fallback_text and fallback_text != text:
            data = render(fallback_text, allow_network=False) or render(fallback_text)
            spoken = fallback_text
            if data is not None:
                robot_log.event("audio.error", logging.WARNING, stage="tts-fallback",
                                err="synthesis unavailable", spoke=fallback_text)
        if data is None:
            return False

        ok = _play_wav(data)

    if ok and guard and config.TTS_GUARD_S > 0:
        time.sleep(config.TTS_GUARD_S)
    return ok and spoken is not None


def connected() -> bool:
    """Session is up. Say this BEFORE the microphone stream opens, so it cannot
    be streamed to the model as if the operator had spoken."""
    return say(STATIC_PHRASES["connected"], event="voice.connected")


def error(description: str = "") -> bool:
    """Session has died. Say this AFTER it is torn down, for the same reason in
    reverse: by then nothing is listening.

    `description` is the point of the whole rewrite — a tone could only say
    "something broke", where this says which thing.
    """
    generic = STATIC_PHRASES["error"]
    if not description:
        return say(generic, event="voice.error", guard=False)
    text = f"Voice session failed. {description.rstrip('.')}. Check the log."
    return say(text, event="voice.error", guard=False,
               fallback_text=generic, detail=description)


def prime(names: Optional[list] = None) -> dict:
    """Render the static phrases into the cache. Safe to call at every boot —
    a hit costs a stat() — and it is what makes the offline failure path work.

    Returns {name: bool} so a caller can log which ones are now safe.
    """
    out = {}
    for name in (names or list(STATIC_PHRASES)):
        phrase = STATIC_PHRASES.get(name)
        out[name] = phrase is not None and render(phrase) is not None
    return out


def cache_info() -> dict:
    d = _cache_dir()
    files = list(d.glob("*.wav"))
    return {
        "dir": str(d),
        "files": len(files),
        "bytes": sum(f.stat().st_size for f in files),
        "primed": {n: _cache_path(p).is_file() for n, p in STATIC_PHRASES.items()},
    }


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Audition / prime the robot's voice.")
    ap.add_argument("text", nargs="*", help="text to speak (default: the static phrases)")
    ap.add_argument("--prime", action="store_true", help="cache the static phrases, do not play")
    ap.add_argument("--info", action="store_true", help="show cache state and exit")
    args = ap.parse_args()

    robot_log.setup()

    if args.info:
        print(json.dumps(cache_info(), indent=2))
        raise SystemExit(0)

    print(f"model   : {config.TTS_MODEL}")
    print(f"voice   : {config.TTS_VOICE}")
    print(f"style   : {config.TTS_STYLE or '(none)'}")
    print(f"playback: {backend()}")

    if args.prime:
        for name, ok in prime().items():
            print(f"  {'ok  ' if ok else 'FAIL'} {name}: {STATIC_PHRASES[name]!r}")
        raise SystemExit(0)

    if args.text:
        print("spoke:", say(" ".join(args.text), guard=False))
    else:
        for name, phrase in STATIC_PHRASES.items():
            print(f"  {name}: {phrase!r}")
            say(phrase, guard=False)
            time.sleep(0.4)

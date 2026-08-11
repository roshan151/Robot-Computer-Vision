#!/usr/bin/env python3
"""
Why can't I hear anything?

Audio on this robot fails silently by design — a missing speaker must never
stop the robot driving — which is exactly what makes it annoying to debug. This
walks every layer and says which one is broken.

  python diagnose_audio.py              # report + play the cues and a phrase
  python diagnose_audio.py --quiet      # report only, play nothing
  python diagnose_audio.py --systemd    # simulate the service environment

Run it BOTH ways when the service is the thing that's silent:

  python diagnose_audio.py                                   # as your login user
  sudo systemctl stop robot-voice
  sudo -u roshan151 env -i PATH=/usr/bin:/bin python3 diagnose_audio.py

If the first works and the second does not, the problem is the service
environment (almost always XDG_RUNTIME_DIR), not the audio stack.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

OK, BAD, WARN, INFO = "  ok  ", " FAIL ", " WARN ", "      "
findings: list = []


def note(level: str, msg: str, fix: str = "") -> None:
    print(f"[{level}] {msg}")
    if fix:
        print(f"         -> {fix}")
    if level in (BAD, WARN):
        findings.append((level, msg, fix))


def run(cmd: list, timeout: float = 5.0) -> str:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip()
    except Exception:
        return ""


# --------------------------------------------------------------------------- #

def section(title: str) -> None:
    print(f"\n=== {title} " + "=" * max(0, 60 - len(title)))


def check_env() -> None:
    section("process environment")
    uid = os.getuid()
    print(f"       user={os.environ.get('USER', '?')} uid={uid}")

    xdg = os.environ.get("XDG_RUNTIME_DIR", "")
    if xdg:
        note(OK if Path(xdg).exists() else BAD,
             f"XDG_RUNTIME_DIR={xdg}" + ("" if Path(xdg).exists() else " (missing!)"),
             "" if Path(xdg).exists() else f"the directory does not exist; is uid {uid} logged in?")
    else:
        note(BAD, "XDG_RUNTIME_DIR is not set",
             "A systemd SYSTEM service has no user session, so PulseAudio/PipeWire "
             f"and the Bluetooth sink are invisible. Add to the unit:\n"
             f"            Environment=XDG_RUNTIME_DIR=/run/user/{uid}")

    pulse = os.environ.get("PULSE_SERVER", "")
    print(f"       PULSE_SERVER={pulse or '(unset — will use default)'}")


def check_config() -> None:
    section("config toggles")
    try:
        import config
    except Exception as e:
        note(BAD, f"config failed to import: {type(e).__name__}: {e}")
        return

    note(OK if config.AUDIO_CUES_ENABLED else BAD,
         f"AUDIO_CUES_ENABLED = {config.AUDIO_CUES_ENABLED}",
         "" if config.AUDIO_CUES_ENABLED else "set ROBOT_AUDIO_CUES=1")
    note(OK if config.BATTERY_ANNOUNCE else BAD,
         f"BATTERY_ANNOUNCE   = {config.BATTERY_ANNOUNCE}",
         "" if config.BATTERY_ANNOUNCE else "set ROBOT_BATTERY_ANNOUNCE=1")

    gain = config.AUDIO_CUE_GAIN
    note(WARN if gain < 0.15 else OK, f"AUDIO_CUE_GAIN     = {gain}",
         "very quiet — try 0.5" if gain < 0.15 else "")
    print(f"       AUDIO_CUE_DEVICE   = {config.AUDIO_CUE_DEVICE or '(default device)'}")
    print(f"       ROBOT_SPEECH       = {config.ROBOT_SPEECH_ENABLED}"
          "   (only gates in-conversation speech, not the battery report)")
    print(f"       NIX_TTS_DIR        = {config.NIX_TTS_DIR or '(unset)'}")

    env_file = Path("/etc/robot.env")
    if env_file.exists():
        try:
            for line in env_file.read_text().splitlines():
                if line.startswith(("ROBOT_AUDIO_CUES=0", "ROBOT_BATTERY_ANNOUNCE=0")):
                    note(BAD, f"/etc/robot.env disables it: {line}")
        except PermissionError:
            note(WARN, "/etc/robot.env exists but is not readable by this user",
                 "run this script as the service user to see its effective config")


def check_players() -> None:
    section("playback backends")
    try:
        import numpy  # noqa: F401
        note(OK, "numpy importable")
    except Exception as e:
        note(WARN, f"numpy NOT importable: {type(e).__name__}: {e}",
             "pip install numpy   (sounddevice playback needs it)")
    try:
        import sounddevice as sd
        note(OK, f"sounddevice importable (portaudio {sd.get_portaudio_version()[1]})")
        try:
            default_out = sd.query_devices(kind="output")
            print(f"       default output: {default_out['name']}")
        except Exception as e:
            note(WARN, f"sounddevice has no usable output device: {e}")
    except Exception as e:
        note(WARN, f"sounddevice NOT importable: {type(e).__name__}: {e}",
             "pip install sounddevice   (falls back to aplay/paplay)")

    for exe in ("paplay", "aplay", "espeak-ng", "espeak"):
        path = shutil.which(exe)
        note(OK if path else WARN, f"{exe:10s} {path or 'NOT FOUND'}",
             "" if path else (f"sudo apt install espeak-ng" if "espeak" in exe
                              else "sudo apt install alsa-utils pulseaudio-utils"))


def check_sinks() -> None:
    section("audio sinks (where sound would go)")
    if not shutil.which("pactl"):
        note(WARN, "pactl not available — cannot inspect sinks")
        return

    default = run(["pactl", "get-default-sink"])
    print(f"       default sink: {default or '(unknown)'}")

    sinks = run(["pactl", "list", "sinks"])
    if not sinks:
        note(BAD, "no sinks visible to this process",
             "PulseAudio/PipeWire is unreachable. Under systemd this is almost "
             "always a missing XDG_RUNTIME_DIR.")
        return

    name = vol = mute = ""
    for line in sinks.splitlines():
        s = line.strip()
        if s.startswith("Name:"):
            name = s.split(None, 1)[1]
        elif s.startswith("Volume:") and "front-left" in s and name:
            vol = s.split("/")[1].strip() if "/" in s else s
        elif s.startswith("Mute:") and name:
            mute = s.split(None, 1)[1]
            flag = BAD if mute == "yes" else OK
            note(flag, f"{name}  vol={vol or '?'}  muted={mute}",
                 "unmute: pactl set-sink-mute <name> 0" if mute == "yes" else "")
            name = vol = mute = ""

    cards = run(["pactl", "list", "cards"])
    if "bluez_card" in cards:
        active = [l.strip() for l in cards.splitlines() if "Active Profile" in l]
        for a in active:
            print(f"       {a}")
        if "headset-head-unit" in cards and "a2dp" not in cards.lower():
            note(WARN, "Bluetooth is in HFP (headset) mode",
                 "HFP is 8 kHz and its SCO link takes 100-500 ms to wake, which "
                 "can swallow a 160 ms cue entirely. See the timing test below.")
    else:
        note(WARN, "no Bluetooth card visible",
             "if the cues should come out of the buds, they are not connected: "
             "bluetoothctl connect $BT_MAC")


def check_pisugar() -> None:
    section("battery source")
    try:
        import battery
        st = battery.read(retries=1, delay=0)
        if st.ok:
            note(OK, f"PiSugar: {st.as_dict()}  -> would say: {st.phrase()!r}")
        else:
            note(WARN, "PiSugar unreachable — the robot would say "
                       "'Battery level unknown'",
                 "is pisugar-server running?  systemctl status pisugar-server")
    except Exception as e:
        note(BAD, f"battery module failed: {type(e).__name__}: {e}")


def play_tests(quiet: bool) -> None:
    section("live playback test")
    import config
    from audio_cues import TONES, AudioCues

    c = AudioCues(enabled=True)
    note(OK if c._player != "none" else BAD, f"cue backend selected: {c._player}",
         "" if c._player != "none" else
         "no backend at all — install sounddevice+numpy, or alsa-utils for aplay")

    import speech
    b = speech.backend()
    note(OK if b != "none" else BAD, f"tts backend selected: {b}",
         "" if b != "none" else "sudo apt install espeak-ng")

    if quiet or c._player == "none":
        return

    print("\n       listen now — each cue plays twice, 1 s apart")
    for name in TONES:
        for i in range(2):
            t0 = time.monotonic()
            ok = c.play(name, guard=False)
            dt = time.monotonic() - t0
            expected = len(c._pcm[name]) / 2 / 16000
            flag = OK if ok else BAD
            hint = ""
            if ok and dt < expected * 0.5:
                flag, hint = WARN, ("returned faster than the tone is long — "
                                    "playback is not actually blocking")
            note(flag, f"{name:8s} #{i+1}  {dt:.2f}s (tone is {expected:.2f}s)", hint)
            time.sleep(1.0)

    if b != "none":
        print("\n       speaking the battery phrase")
        import battery
        st = battery.read(retries=1, delay=0)
        t0 = time.monotonic()
        spoke = speech.say(st.phrase().capitalize())
        note(OK if spoke else BAD,
             f"speech.say -> {spoke}  ({time.monotonic() - t0:.2f}s)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quiet", action="store_true", help="report only, play nothing")
    args = ap.parse_args()

    import logging
    logging.basicConfig(level=logging.WARNING,
                        format="       [%(name)s] %(message)s")

    check_env()
    check_config()
    check_players()
    check_sinks()
    check_pisugar()
    play_tests(args.quiet)

    section("verdict")
    if not findings:
        print("       nothing obviously wrong. If you still hear nothing:")
        print("         - the tones played into a sink you are not listening to")
        print("         - raise ROBOT_AUDIO_CUE_GAIN (default 0.25)")
        print("         - HFP link wake can swallow the first tone; the second")
        print("           of each pair above is the one to trust")
    else:
        for level, msg, fix in findings:
            print(f"  [{level}] {msg}")
            if fix:
                print(f"           {fix.splitlines()[0]}")
    print()


if __name__ == "__main__":
    main()

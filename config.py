"""Serial link, motion calibration, secrets, and vision service defaults.

Single source of truth for configuration. Nothing else in the tree should call
os.environ directly — if a setting matters, it gets a name here.

Secrets are READ here, never STORED here. This file is committed; the values
live in .env (gitignored) or /etc/robot.env (systemd). See .env.example.
"""

import os
from pathlib import Path

# Load .env before anything reads a value, so imports in any order behave the
# same. Optional: on the Pi the systemd unit supplies the environment instead
# (EnvironmentFile=/etc/robot.env), and python-dotenv may not be installed at
# all in a minimal deployment.
try:
    from dotenv import load_dotenv

    load_dotenv(Path(__file__).resolve().parent / ".env")
except Exception:  # pragma: no cover - absence is a valid deployment
    pass

# USB serial from Arduino (often /dev/ttyUSB0 or /dev/ttyACM0).
# On macOS this is a /dev/cu.usbserial-* name — set ROBOT_SERIAL_PORT rather
# than editing this, so the same checkout runs on the Pi and on a laptop.
SERIAL_PORT = os.environ.get("ROBOT_SERIAL_PORT", "/dev/ttyUSB0")
BAUD_RATE = int(os.environ.get("ROBOT_SERIAL_BAUD", "115200"))

# Heartbeat: must be comfortably faster than the firmware's link watchdog
# (LINK_TIMEOUT_MS = 1000 ms in drivetrain.ino).  At 0.25 s the firmware
# would need to miss four consecutive heartbeats before braking.
PING_INTERVAL_S = float(os.environ.get("ROBOT_PING_INTERVAL_S", "0.25"))

# ---------------------------------------------------------------------------
# Framed-protocol settings (firmware v3)
# ---------------------------------------------------------------------------
# How long to wait for a command's A/N reply before retransmitting it.
ACK_TIMEOUT_S = float(os.environ.get("ROBOT_ACK_TIMEOUT_S", "0.35"))

# Retransmissions per command (same sequence number — the firmware
# deduplicates, so retries never double-execute).
CMD_RETRIES = int(os.environ.get("ROBOT_CMD_RETRIES", "3"))

# Host-side ceiling on one encoder-counted move.  Must exceed the
# firmware's own MOVE_TIMEOUT_MS (15 s) so the firmware's D report,
# not a host timeout, is the normal failure path.
MOVE_TIMEOUT_S = float(os.environ.get("ROBOT_MOVE_TIMEOUT_S", "20.0"))

# Optional read timeout for non-blocking serial reads (seconds)
SERIAL_TIMEOUT = 0.05

# ---------------------------------------------------------------------------
# Encoder calibration — tune these on flat ground before running missions.
# ---------------------------------------------------------------------------
# How to calibrate TICKS_PER_CM:
#   1. Mark a start line on the floor.
#   2. Run: drivetrain.straight_m(1.0)
#   3. Measure actual distance traveled in cm.
#   4. New value = current TICKS_PER_CM * (100 / measured_cm)
TICKS_PER_CM: float = float(os.environ.get("ROBOT_TICKS_PER_CM", "25.0"))

# How to calibrate TICKS_PER_DEGREE:
#   1. Run: drivetrain.right(360)
#   2. Count actual degrees rotated.
#   3. New value = current TICKS_PER_DEGREE * (360 / measured_degrees)
TICKS_PER_DEGREE: float = float(os.environ.get("ROBOT_TICKS_PER_DEGREE", "3.5"))

# Acceptable sync-error ratio between left and right encoders (0.0–1.0).
# A move producing more skew than this triggers a warning log.
# 0.10 = allow up to 10% difference between wheels.
ENCODER_SYNC_WARN_RATIO: float = float(os.environ.get("ROBOT_SYNC_WARN_RATIO", "0.10"))

# The firmware emits ENC: telemetry every 100 ms and zeroes both counters at
# the start of each encoder-counted move.  After a move ACKs, wait at least one
# telemetry interval before reading the counts, otherwise the last line
# received is a mid-move sample and the reported travel is short.
ENCODER_SETTLE_S: float = float(os.environ.get("ROBOT_ENCODER_SETTLE_S", "0.15"))

# ---------------------------------------------------------------------------
# Connection / handshake settings
# ---------------------------------------------------------------------------
# How long to wait after a DTR reset before draining the buffer and retrying
# the handshake. The UNO bootloader takes ~1.5 s; 2.5 s gives real margin.
# Only used in the DTR-reset fallback path — the fast-path ping skips this.
ARDUINO_DRAIN_WAIT_S: float = float(os.environ.get("ROBOT_DRAIN_WAIT_S", "2.5"))

# Number of times to retry the initial handshake "S" command before giving up.
HANDSHAKE_RETRIES: int = int(os.environ.get("ROBOT_HANDSHAKE_RETRIES", "3"))

# Per-attempt timeout for the handshake ping (seconds).
HANDSHAKE_TIMEOUT_S: float = float(os.environ.get("ROBOT_HANDSHAKE_TIMEOUT_S", "2.0"))

# ---------------------------------------------------------------------------
# Motion defaults
# ---------------------------------------------------------------------------
DEFAULT_TURN_DEGREES = float(os.environ.get("ROBOT_DEFAULT_TURN_DEG", "90"))
DEFAULT_MOVE_METERS = float(os.environ.get("ROBOT_DEFAULT_MOVE_M", "1.0"))

# Default PWM duty for every encoder-counted move, as a percentage of full
# scale.  Measured on hardware: right(30) completes in < 1.2 s at 70 %, which
# is the budget the gesture channel needs (a NO gesture is three turns).
#
# Ramping is NOT done here.  The firmware already slews PWM at RAMP_STEP/RAMP_MS
# = 0.5 PWM per ms (drivetrain.ino), so 70 % (=178 PWM) is reached in ~356 ms
# from rest, and softStop() ramps down at 2x that rate before engaging the
# brake.  That is what protects the supply rail from the inrush/back-EMF dip
# that resets the board.  Adding a second ramp on the host would fight it.
DEFAULT_SPEED_PERCENT = float(os.environ.get("ROBOT_DEFAULT_SPEED_PCT", "70.0"))

# ---------------------------------------------------------------------------
# Audio cues + listening
# ---------------------------------------------------------------------------
# The robot runs headless, so a short tone is the only way to know it is
# waiting. Tones only — never speech. See audio_cues.py for why ordering
# (cue first, THEN open the microphone) is what keeps it out of the input.
AUDIO_CUES_ENABLED = os.environ.get("ROBOT_AUDIO_CUES", "1") not in ("0", "false", "no")
# Spoken output during conversation. Off by design — the robot answers with
# gestures, and its own voice in the microphone is the failure mode the whole
# design avoids. This does NOT gate the startup battery report or failure
# audio: those play when no capture stream is open, so they are always safe.
ROBOT_SPEECH_ENABLED = os.environ.get("ROBOT_SPEECH", "0") in ("1", "true", "yes")
SPEECH_WPM = int(os.environ.get("ROBOT_SPEECH_WPM", "150"))
SPEECH_AMPLITUDE = int(os.environ.get("ROBOT_SPEECH_AMPLITUDE", "120"))

# ---------------------------------------------------------------------------
# Battery (PiSugar)
# ---------------------------------------------------------------------------
# Spoken once at startup, before the microphone opens. On a headless robot a
# flat battery is otherwise invisible until the Pi browns out mid-drive — which
# on this board also resets the Arduino.
BATTERY_ANNOUNCE = os.environ.get("ROBOT_BATTERY_ANNOUNCE", "1") not in ("0", "false", "no")
BATTERY_LOW_PCT = float(os.environ.get("ROBOT_BATTERY_LOW_PCT", "20"))
PISUGAR_SOCKETS = tuple(
    s.strip() for s in os.environ.get(
        "PISUGAR_SOCKETS", "/tmp/pisugar-server.sock,/tmp/pisugar.sock"
    ).split(",") if s.strip()
)
PISUGAR_TCP = (
    os.environ.get("PISUGAR_HOST", "127.0.0.1"),
    int(os.environ.get("PISUGAR_PORT", "8423")),
)
AUDIO_CUE_DEVICE = os.environ.get("ROBOT_AUDIO_CUE_DEVICE", "")
AUDIO_CUE_GAIN = float(os.environ.get("ROBOT_AUDIO_CUE_GAIN", "0.25"))
# Settle time after a cue before capture opens, covering the room's reverb
# tail. Raise it if the first syllable of a command goes missing.
AUDIO_CUE_GUARD_S = float(os.environ.get("ROBOT_AUDIO_CUE_GUARD_S", "0.15"))

# How long one listen() call waits for speech to START before looping.
# This is NOT a prompt interval: a timeout re-arms the microphone silently,
# with no cue and no output. It exists only so a wedged capture device can be
# distinguished from an idle one — an infinite block would hang forever with
# nothing in the log. Raise it to make the robot more patient; it never
# changes what the operator hears.



# ---------------------------------------------------------------------------
# Live agent
# ---------------------------------------------------------------------------
# The session streams audio continuously and the model calls the robot's
# functions directly. There is no push-to-listen, no per-utterance upload and
# no ambient calibration — those existed to decide when to spend a request,
# and a streaming session has no discrete requests to spend.

# AUDIO is the only value the native-audio Live models accept — they are
# speech-to-speech and reject TEXT with "1007 ... response modalities (TEXT)
# is not supported by the model".
#
# The robot is silent anyway. This controls what the model GENERATES, not what
# gets played: live_agent reads the returned PCM off the socket and drops it,
# so no speaker emits it and the open microphone never hears it. The model's
# words still reach logs.json via output_audio_transcription.
LIVE_RESPONSE_MODALITY = os.environ.get("ROBOT_LIVE_MODALITY", "AUDIO").upper()

# Play the model's speech instead of discarding it. Off by design: the session
# holds the microphone open continuously, so anything played is streamed
# straight back to the model as if the operator had said it.
LIVE_PLAY_AUDIO = os.environ.get("ROBOT_LIVE_PLAY_AUDIO", "0") in ("1", "true", "yes")

# Capture device for the uplink; blank means the system default.
AUDIO_INPUT_DEVICE = os.environ.get("ROBOT_AUDIO_INPUT_DEVICE", "")

# Reconnect backoff after a dropped session, doubling to the cap.
LIVE_RECONNECT_BACKOFF_S = float(os.environ.get("ROBOT_LIVE_BACKOFF_S", "2.0"))
LIVE_RECONNECT_MAX_S = float(os.environ.get("ROBOT_LIVE_BACKOFF_MAX_S", "60.0"))

# Hard ceiling on a single drive call, enforced in the tool layer rather than
# the prompt: a limit the model can talk itself out of is not a limit.
MAX_DRIVE_METERS = float(os.environ.get("ROBOT_MAX_DRIVE_M", "5.0"))

LIVE_SYSTEM_PROMPT = os.environ.get("ROBOT_LIVE_PROMPT", """\
You are Robin, a small wheeled robot. You hear the operator continuously and
act by calling your functions. Do not narrate; call the function.

You have no voice and no screen. Your only reply is movement:
  answer("yes")      nods
  answer("no")       shakes
  answer("unclear")  the same shake as "no" - you could not make out the speech

Rules that matter:
  - Call stop() the instant you hear "stop", and whenever you are unsure
    whether it is safe to keep moving. A needless stop costs nothing.
  - Never guess a movement you are unsure of. The robot drives on a floor with
    obstacles it cannot see. If you did not understand, answer("unclear").
  - Ignore speech that is not addressed to you, and background conversation.
  - Defaults when no number is given: 1 metre, 90 degrees.
  - turn() takes positive degrees for RIGHT, negative for LEFT.
    drive() takes positive metres for FORWARD, negative for BACKWARD.
""")

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
# The robot has no verbal feedback, so this file is the only place a fault is
# ever explained. JSON Lines — one object per line, appendable, and a process
# killed mid-write costs one record instead of the whole file.
LOG_PATH = os.environ.get("ROBOT_LOG_PATH", "logs.json")
LOG_MAX_BYTES = int(os.environ.get("ROBOT_LOG_MAX_BYTES", "2000000"))
LOG_BACKUPS = int(os.environ.get("ROBOT_LOG_BACKUPS", "3"))

# ---------------------------------------------------------------------------
# Emergency stop
# ---------------------------------------------------------------------------
# ArduinoBridge.move() holds _cmd_lock for the whole blocking move, so a normal
# stop() issued from another thread cannot interrupt it — it waits for the lock
# and arrives after the move has already finished.  emergency_stop() bypasses
# _cmd_lock and writes an out-of-band S frame directly.
#
# The firmware deduplicates on the single previous sequence number, so the
# e-stop's seq must differ from the in-flight move's.  Reserve the top of the
# range for e-stops and cap the normal counter below it.
ESTOP_SEQ_MIN = 240
ESTOP_SEQ_MAX = 255
NORMAL_SEQ_MAX = ESTOP_SEQ_MIN - 1     # normal commands use 0..239

# ---------------------------------------------------------------------------
# Gesture vocabulary — the robot's only output channel during normal operation
# ---------------------------------------------------------------------------
# The robot never speaks.  It answers by moving.  Audio is reserved for the
# failure path, and only ever plays once the voice session is already torn down.
GESTURE_YES_METERS = float(os.environ.get("ROBOT_GESTURE_YES_M", "0.1"))
GESTURE_NO_DEGREES = float(os.environ.get("ROBOT_GESTURE_NO_DEG", "30.0"))

# Vision HTTP API (run Vision service: uvicorn Vision.app:app --host 0.0.0.0 --port 8080)
VISION_SERVICE_URL = os.environ.get(
    "VISION_SERVICE_URL",
    "http://127.0.0.1:8080/detect_objects:frame",
)

# Comma-separated class names; if any appear, guardian may stop the robot while moving
VISION_HALT_OBJECTS = [
    s.strip()
    for s in os.environ.get("VISION_HALT_OBJECTS", "").split(",")
    if s.strip()
]

# Guardian poll rate (Hz) while robot reports motion
VISION_GUARD_HZ = float(os.environ.get("VISION_GUARD_HZ", "4.0"))

# ---------------------------------------------------------------------------
# Secrets
# ---------------------------------------------------------------------------
# Values come from the environment or .env — never from this file, which is
# committed. Missing secrets resolve to "" rather than raising at import time,
# so tests and offline tools can import config without credentials present.
# Call require() at the point of use to fail with a message that names the
# variable and where to put it.
#
# Source order (first non-empty wins), per secret:
#   1. process environment  (systemd EnvironmentFile=/etc/robot.env)
#   2. .env next to this file
#   3. ""  -> require() raises with instructions

def _first_env(*names: str) -> str:
    for n in names:
        v = os.environ.get(n)
        if v:
            return v.strip()
    return ""


# Google Gemini — the only voice backend.
GEMINI_API_KEY = _first_env("GEMINI_API_KEY", "GOOGLE_API_KEY")
GEMINI_LIVE_MODEL = os.environ.get(
    "GEMINI_LIVE_MODEL", "gemini-3.1-flash-live-preview")

# Nix TTS — used only on the failure path (pre-rendered clips), never in
# normal operation. Paths, not secrets, but same principle: no hardcoded
# home directories in the tree.
NIX_TTS_DIR = os.environ.get("NIX_TTS_DIR", "")
NIX_TTS_MODEL = os.environ.get("NIX_TTS_MODEL", "")

# Bluetooth headset (see check_bt_audio.sh).
BT_MAC = os.environ.get("BT_MAC", "")

# Every name here is treated as sensitive by the log redactor.
SECRET_NAMES = (
    "GEMINI_API_KEY",
)

# ---------------------------------------------------------------------------
# Voice backend
# ---------------------------------------------------------------------------
# Gemini only, deliberately. It takes the microphone audio and the planning
# prompt in ONE request and returns structured JSON, replacing the old
# two-hop  mic -> Google Web Speech -> text -> OpenAI  pipeline.
#
# The toggle exists so the value is named and validated rather than implied,
# and so a future backend has an obvious place to land. Anything other than
# "gemini" is rejected at import — a silently ignored setting is worse than
# no setting.
VOICE_BACKENDS = ("gemini",)
VOICE_BACKEND = os.environ.get("ROBOT_VOICE_BACKEND", "gemini").strip().lower()
if VOICE_BACKEND not in VOICE_BACKENDS:
    raise ValueError(
        f"ROBOT_VOICE_BACKEND={VOICE_BACKEND!r} is not supported. "
        f"Supported: {', '.join(VOICE_BACKENDS)}."
    )

# gemini-3.6-flash is the current Flash generation; audio in, JSON out.
GEMINI_MODEL = os.environ.get("GEMINI_MODEL", "gemini-3.6-flash")
GEMINI_TEMPERATURE = float(os.environ.get("GEMINI_TEMPERATURE", "0.2"))
# Prior turns kept as text. Audio is never resent — the transcript carries what
# the planner needs at a fraction of the tokens.
GEMINI_HISTORY_TURNS = int(os.environ.get("GEMINI_HISTORY_TURNS", "6"))
# Inline audio ceiling. The API limit is 20 MB for the whole request; this is a
# much tighter sanity bound, since a 12 s command at 16 kHz mono is ~384 kB and
# anything far larger means the recorder is misconfigured.
GEMINI_MAX_AUDIO_KB = int(os.environ.get("GEMINI_MAX_AUDIO_KB", "4096"))
# Gemini downsamples to 16 kbps mono regardless, so sending more is wasted
# upload on the Pi's WiFi.
GEMINI_AUDIO_RATE = int(os.environ.get("GEMINI_AUDIO_RATE", "16000"))


class MissingSecret(RuntimeError):
    """A required credential is not configured."""


def require(name: str) -> str:
    """Return a secret, or raise with instructions naming the variable.

    Preferred over reading the constant directly, so a missing key produces
    one clear line in logs.json instead of a 401 from a vendor SDK three
    frames deep.
    """
    value = globals().get(name, "")
    if not value:
        raise MissingSecret(
            f"{name} is not set. Provide it in one of:\n"
            f"  - {Path(__file__).resolve().parent / '.env'}  (development)\n"
            f"  - /etc/robot.env                              (systemd service)\n"
            f"  - the process environment\n"
            f"See .env.example for the full list."
        )
    return str(value)


def secret_values() -> tuple:
    """Non-empty secret values, for redaction. Never log the result."""
    return tuple(v for v in (globals().get(n, "") for n in SECRET_NAMES) if v)

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

# Straight-line trim, parts per thousand. Applied to the firmware at connect.
#
# The firmware sync loop equalises encoder TICKS. Equal ticks is not equal
# DISTANCE when the wheels differ in effective rolling radius, so the robot can
# curve while both encoders report a perfect match — and a P-only loop leaves
# steady-state error besides. This feedforward bias cancels both.
#
#   POSITIVE slows the LEFT wheel  -> corrects veering RIGHT
#   NEGATIVE slows the RIGHT wheel -> corrects veering LEFT
#
# How to find your value:  python tests/calibrate_straight.py
SYNC_TRIM_PPT: int = int(os.environ.get("ROBOT_SYNC_TRIM_PPT", "0"))

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
LISTEN_TIMEOUT_S = float(os.environ.get("ROBOT_LISTEN_TIMEOUT_S", "300"))
# Maximum length of a single spoken command, once speech has begun.
LISTEN_PHRASE_LIMIT_S = float(os.environ.get("ROBOT_LISTEN_PHRASE_S", "8"))

# Silence that ends an utterance. This is dead time the operator waits through
# on EVERY command, before the request is even sent — it is felt as latency
# just as much as the API call is. It also trims trailing silence off the
# upload. SpeechRecognition's default is 0.8 s.
# Too low and it cuts you off mid-sentence between words.
LISTEN_PAUSE_S = float(os.environ.get("ROBOT_LISTEN_PAUSE_S", "0.5"))
# One-off ambient noise calibration at startup (seconds). Per-turn calibration
# would add this much dead air to every single command.
LISTEN_CALIBRATE_S = float(os.environ.get("ROBOT_LISTEN_CALIBRATE_S", "1.0"))

# Floor under the recognizer's energy threshold.
#
# dynamic_energy_threshold keeps adapting to the room, which is what stops one
# startup calibration going stale — but in a QUIET room it adapts downward
# without limit until the microphone triggers on the Pi's own fan. Every such
# trigger became an upload, and enough of them became a 429.
#
# Re-applied before every listen, so the drift can never go below it. Raise it
# if the robot still wakes on nothing; lower it if quiet speech is missed.
LISTEN_MIN_ENERGY = float(os.environ.get("ROBOT_LISTEN_MIN_ENERGY", "300"))

# ---------------------------------------------------------------------------
# Local speech gate — what stops noise becoming API requests
# ---------------------------------------------------------------------------
# Recognizer.listen() detects ENERGY, not speech. This second, cheap, local
# check runs on the captured clip and drops anything that cannot plausibly be a
# spoken command, before it costs a request. See audio_gate.py.
#
# Tune against real recordings:  python audio_gate.py clip1.wav clip2.wav
GATE_ENABLED = os.environ.get("ROBOT_GATE", "1") not in ("0", "false", "no")
# A spoken command is at least a few hundred ms. A click is ~50 ms.
GATE_MIN_DURATION_S = float(os.environ.get("ROBOT_GATE_MIN_DUR_S", "0.35"))
# Loudest 20 ms frame, 0-1. Below this it is room tone.
GATE_MIN_PEAK = float(os.environ.get("ROBOT_GATE_MIN_PEAK", "0.012"))
# Seconds of frames near the peak. Rejects transients: one loud frame
# surrounded by silence is a bump, not a word.
GATE_MIN_VOICED_S = float(os.environ.get("ROBOT_GATE_MIN_VOICED_S", "0.20"))
# Peak-to-median frame energy. Speech varies at syllable rate; a fan or motor
# is loud, sustained and FLAT — the one case an energy threshold cannot reject.
GATE_MIN_MODULATION = float(os.environ.get("ROBOT_GATE_MIN_MODULATION", "2.5"))

# ---------------------------------------------------------------------------
# Request budget
# ---------------------------------------------------------------------------
# Hard client-side ceiling, independent of what triggers the microphone. The
# gate should prevent runaway uploads; this guarantees it, so a pathological
# room cannot burn the quota no matter what.
# Gemini's free tier is commonly 15 RPM — check your own limit and set this
# slightly below it.
GEMINI_MAX_RPM = int(os.environ.get("GEMINI_MAX_RPM", "12"))
# Minimum gap between two requests, so a burst cannot fire back to back.
GEMINI_MIN_INTERVAL_S = float(os.environ.get("GEMINI_MIN_INTERVAL_S", "1.0"))
# How long to stop calling after a 429, doubling each consecutive one.
GEMINI_COOLDOWN_S = float(os.environ.get("GEMINI_COOLDOWN_S", "20"))
GEMINI_COOLDOWN_MAX_S = float(os.environ.get("GEMINI_COOLDOWN_MAX_S", "300"))

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
GEMINI_LIVE_MODEL = os.environ.get("GEMINI_LIVE_MODEL", "gemini-live-2.5-flash-preview")

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

# THE LATENCY KNOB. Gemini 3.x models reason internally before answering, and
# on by default that costs many seconds — measured 5-19 s round trips for
# "go forward", with no correlation to audio length because the time was spent
# thinking, not transcribing.
#
# "minimal" is the level built for latency-sensitive work. Turning a spoken
# movement command into two JSON fields needs no deliberation.
# Levels: minimal | low | medium | high
GEMINI_THINKING_LEVEL = os.environ.get("GEMINI_THINKING_LEVEL", "minimal")

# The reply is a transcript plus a couple of steps. Capping this stops a
# confused model from spending seconds generating tokens nobody reads.
GEMINI_MAX_OUTPUT_TOKENS = int(os.environ.get("GEMINI_MAX_OUTPUT_TOKENS", "512"))

# Transient server failures (500/503/504) are retried automatically with the
# SAME audio. Without this the operator has to repeat the command by hand —
# which in one 6-minute session was 3 of 13 commands.
GEMINI_RETRIES = int(os.environ.get("GEMINI_RETRIES", "2"))
GEMINI_RETRY_BACKOFF_S = float(os.environ.get("GEMINI_RETRY_BACKOFF_S", "0.6"))
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
